//package com.example.ftclibexamples;
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
//@Disabled
public class DeadWheelFunctions extends LinearOpMode {

    public static final double TRACKWIDTH = 28.6; //said 14.7
    public static final double CENTER_WHEEL_OFFSET = -11.321628; //used to be -2.1
    public static final double WHEEL_DIAMETER = 3.5; //used to say 2.0
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    private DcMotorEx motorFL, motorFR, motorBL, motorBR;
    private MotorEx motorFLenc, motorFRenc, motorBLenc, motorBRenc;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private IMU imu;
    double previousHeading = 0;
    double processedHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double currentTime = timer.milliseconds();
        double oldTime;

        motorFLenc = new MotorEx(hardwareMap, "motorFLandStrafeOdo");
        motorFRenc = new MotorEx(hardwareMap, "motorFRandForwardOdo"); //also has right odometer
        motorBLenc = new MotorEx(hardwareMap, "motorBLandBackwardOdo");
        motorBRenc = new MotorEx(hardwareMap, "motorBRandLiftEncoder");

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandBackwardOdo");

        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOdometer = motorBLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said front left
        rightOdometer = motorFRenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said front right
        centerOdometer = motorFLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said back left

        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry.updatePose(new Pose2d()); //could probably give it a different starting position to make it use field-related coordinates

        boolean goodPose = false;

        previousHeading = newGetHeading();
        processedHeading = previousHeading;

        waitForStart();

        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            oldTime = currentTime;
            goodPose = placeAndHeading(30, 30, 90, 0.5, 1, 1);
            odometry.updatePose(); // update the position
            telemetry.addData("pos", odometry.getPose());
            telemetry.addData("leftOdometerEncoder", motorBL.getCurrentPosition());
            telemetry.addData("rightOdometerEncoder", motorFR.getCurrentPosition());
            telemetry.addData("centerOdometerEncoder", motorFL.getCurrentPosition());
            telemetry.addData("goodPose", goodPose);
            currentTime = timer.milliseconds();
            telemetry.addData("loop time in ms", currentTime - oldTime);
            telemetry.update();

        }
    }
    public boolean travelByNewPos(double x, double y, double powerMult, double tolerance) { //returns true if it's at the position.
        double currentX = -odometry.getPose().getY();
        double currentY = odometry.getPose().getX();
        double diagonalError = Math.sqrt(Math.pow((currentX - x), 2) + Math.pow((currentX - x), 2));
        double constant;
        if (diagonalError > 10) {
            constant = 1;
        } else {
            constant = diagonalError * 0.05;
        }
        if (constant < 0.3) {constant = 0.3;}

        double angle = getAngleToTravel(currentX, currentY, x, y);
        double BLFRPower;
        double BRFLPower;
        if (Math.abs(x - currentX) > tolerance || Math.abs(y - currentY) > tolerance) { //if not completely in right position
            if (angle >= 0 && angle <= Math.PI / 2) { //quadrant 1
                BRFLPower = 1;
                BLFRPower = angle * 4 / (Math.PI) - 1; //should range from 1 to -1
            } else if (angle >= Math.PI / 2 && angle <= Math.PI) { //quadrant 2
                BLFRPower = 1;
                BRFLPower = (Math.PI / 2 - angle) * 4 / (Math.PI) + 1;
            } else if (angle >= Math.PI && angle <= Math.PI * (1.5)) { //quadrant 3
                BRFLPower = -1;
                BLFRPower = (Math.PI - angle) * 4 / (Math.PI) + 1;

            } else { //quadrant 4
                BLFRPower = -1;
                BRFLPower = (angle - (1.5 * Math.PI)) * 4 / (Math.PI) - 1;
            }
            /*telemetry.addData("BRFL", BRFLPower);
            telemetry.addData("BLFR", BLFRPower);
            telemetry.addData("angle", angle);
            telemetry.update();*/

            motorFL.setPower(BRFLPower * powerMult * constant);
            motorBR.setPower(BRFLPower * powerMult * constant);
            motorBL.setPower(BLFRPower * powerMult * constant);
            motorFR.setPower(BLFRPower * powerMult * constant);
            return false;
        } else {
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            return true;
        }
    }
    //powersXY returns [powerFL, powerBR, powerBL, powerFR, diagonalError]
    public double[] powersXY(double x, double y, double powerMult, double tolerance) {
        double currentX = -odometry.getPose().getY();
        double currentY = odometry.getPose().getX();
        double diagonalError = Math.sqrt(Math.pow((currentX - x), 2) + Math.pow((currentX - x), 2));
        double constant;
        if (diagonalError > 10) {
            constant = 1;
        } else {
            constant = diagonalError * 0.05;
        }
        if (constant < 0.3) {constant = 0.3;}

        double angle = getAngleToTravel(currentX, currentY, x, y);
        double BLFRPower;
        double BRFLPower;
        if (Math.abs(x - currentX) > tolerance || Math.abs(y - currentY) > tolerance) { //if not completely in right position
            if (angle >= 0 && angle <= Math.PI / 2) { //quadrant 1
                BRFLPower = 1;
                BLFRPower = angle * 4 / (Math.PI) - 1; //should range from 1 to -1
            } else if (angle >= Math.PI / 2 && angle <= Math.PI) { //quadrant 2
                BLFRPower = 1;
                BRFLPower = (Math.PI / 2 - angle) * 4 / (Math.PI) + 1;
            } else if (angle >= Math.PI && angle <= Math.PI * (1.5)) { //quadrant 3
                BRFLPower = -1;
                BLFRPower = (Math.PI - angle) * 4 / (Math.PI) + 1;

            } else { //quadrant 4
                BLFRPower = -1;
                BRFLPower = (angle - (1.5 * Math.PI)) * 4 / (Math.PI) - 1;
            }

            double[] returnArray = new double[5]; // 0 = motorFL, 1 = motorBR, 2 = motorBL, 3 = motorFR, 4 = error
            returnArray[0] = BRFLPower * powerMult * constant;
            returnArray[1] = BRFLPower * powerMult * constant;
            returnArray[2] = BLFRPower * powerMult * constant;
            returnArray[3] = BLFRPower * powerMult * constant;
            returnArray[4] = diagonalError;
            return returnArray;
        } else {
            double[] returnArray = new double[5]; // 0 = motorFL, 1 = motorBR, 2 = motorBL, 3 = motorFR, 4 = error
            returnArray[0] = 0.0;
            returnArray[1] = 0.0;
            returnArray[2] = 0.0;
            returnArray[3] = 0.0;
            returnArray[4] = diagonalError;
            return returnArray;
        }
    }
    public double getAngleToTravel(double currentX, double currentY, double endX, double endY) { //returns angle from 0 to 2pi. works well
        double x = endX - currentX;
        double y = endY - currentY;
        double angle = Math.atan(y/x); //range of this is -pi/2 to pi/2
        if (x < 0) { // we are in 2nd or 3rd quadrant
            angle += Math.PI;
        }
        if (angle < 0) {
            angle += Math.PI*2;
        }
        return angle;
    }
    public double newGetHeading(){
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double headingChange = currentHeading - previousHeading;
        if(headingChange < -180){
            headingChange += 360;
        }else if(headingChange > 180){
            headingChange -= 360;
        }
        processedHeading += headingChange;
        previousHeading = currentHeading;
        return processedHeading;
    }
    public boolean absoluteHeading(double idealHeading, double powerMult, double tolerance) { //returns true if it's at the position
        processedHeading = newGetHeading();
        double error = Math.abs(processedHeading % 360 - idealHeading);
        double sign = error/(processedHeading % 360 - idealHeading);
        if (error < tolerance) {
            motorBL.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            return true;
        } else {
            double constant;
            if (error > 45) {
                constant = powerMult;
            } else {
                constant = error/45 * powerMult;
            }
            if (constant < 0.125) {constant = 0.125;}
            motorFL.setPower(constant * sign);
            motorBL.setPower(constant * sign);
            motorFR.setPower(constant * sign * -1);
            motorBR.setPower(constant * sign * -1);
            return false;
        }
    }
    public double[] powersHeading(double idealHeading, double powerMult, double tolerance) { //returns true if it's at the position
        processedHeading = newGetHeading();
        double error = Math.abs(processedHeading % 360 - idealHeading);
        double sign = error/(processedHeading % 360 - idealHeading);
        double[] returnPowers = new double[5]; //0 is FL, 1 is BR, 2 is BL, 3 is FR, 4 is error
        if (error < tolerance) {
            returnPowers[0] = 0.0;
            returnPowers[1] = 0.0;
            returnPowers[2] = 0.0;
            returnPowers[3] = 0.0;
            return returnPowers;
        } else {
            double constant;
            if (error > 45) {
                constant = powerMult;
            } else {
                constant = error/45 * powerMult;
            }
            if (constant < 0.125) {constant = 0.125;}
            //0 is FL, 1 is BR, 2 is BL, 3 is FR, 4 is error
            returnPowers[0] = constant * sign;
            returnPowers[1] = constant * sign * -1;
            returnPowers[2] = constant * sign;
            returnPowers[3] = constant * sign * -1;
            return returnPowers;
        }
    }
    public boolean placeAndHeading(double x, double y, double idealHeading, double powerMult, double cmTol, double degTol) {
        //powersXY returns [powerFL, powerBR, powerBL, powerFR, diagonalError]
        double[] xypow = powersXY(x, y, powerMult, cmTol);
        double[] headingpow = powersHeading(idealHeading, powerMult, degTol);
        if (xypow[4] < cmTol && headingpow[4] < degTol) {
            motorFR.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
            motorFL.setPower(0);
            return true;
        } else {
            double flPower = powerMult * (((xypow[0] * xypow[5]/20) + (headingpow[0] * headingpow[5]/360))/(xypow[5]/20 + headingpow[5]/360));
            if (flPower < 0.2) {flPower = 0.2;}
            double brPower = powerMult * (((xypow[1] * xypow[5]/20) + (headingpow[1] * headingpow[5]/360))/(xypow[5]/20 + headingpow[5]/360));
            if (brPower < 0.2) {brPower = 0.2;}
            double blPower = powerMult * (((xypow[2] * xypow[5]/20) + (headingpow[2] * headingpow[5]/360))/(xypow[5]/20 + headingpow[5]/360));
            if (blPower < 0.2) {blPower = 0.2;}
            double frPower = powerMult * (((xypow[3] * xypow[5]/20) + (headingpow[3] * headingpow[5]/360))/(xypow[5]/20 + headingpow[5]/360));
            if (frPower < 0.2) {frPower = 0.2;}
            motorFL.setPower(flPower);
            motorBR.setPower(brPower);
            motorBL.setPower(blPower);
            motorFR.setPower(frPower);
            return false;
        }
    }


}