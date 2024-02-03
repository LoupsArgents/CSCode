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
public class CSOdoFunctions extends LinearOpMode {

    public static final double TRACKWIDTH = 29.9; //30.04 is too high, 25 is too low, 28 is too low, 29 is too low, 29.52 is slightly too low, 29.7 is slightly too low (0.01 error on a 90 degree turn), 29.8 is slightly low
    public static final double CENTER_WHEEL_OFFSET = -23.5;
    //^was -19.943815 (x error of 10, heading of 0.02)
    //-18 (x error of 12, y error of 2, heading error of 0.02-ish)
    //-22 (x error of 5.26, y error of -2, heading error of -0.002)
    //-24 (x is -4, y is -2, heading error is very small)
    //-23 had (x is 6.5, y is 0.8, heading error is tiny)
    public static final double WHEEL_DIAMETER = 3.5;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    private DcMotorEx motorFL, motorFR, motorBL, motorBR;
    private MotorEx motorFLenc, motorFRenc, motorBLenc, motorBRenc;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private IMU imu;
    double previousHeading = 0;
    double processedHeading = 0;
    double startedAtX = 0;
    double startedAtY = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double currentTime = timer.milliseconds();
        double oldTime;

        motorFLenc = new MotorEx(hardwareMap, "motorFLandStrafeOdo");
        motorFRenc = new MotorEx(hardwareMap, "motorFRandForwardEncoder"); //also has right odometer
        motorBLenc = new MotorEx(hardwareMap, "motorBLandForwardOdo");
        motorBRenc = new MotorEx(hardwareMap, "motorBRandLiftEncoder");

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardEncoder");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandForwardOdo");

        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOdometer = motorBLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = motorFRenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = motorFLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

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
        telemetry.addData("status", "initialized");
        telemetry.addData("x", -odometry.getPose().getY());
        telemetry.addData("y", -odometry.getPose().getX());
        telemetry.addData("odoHeading", odometry.getPose().getHeading());
        telemetry.addData("realHeading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.update();

        double initialX = -odometry.getPose().getY();
        double initialY = -odometry.getPose().getX();

        waitForStart();

        timer.reset();

        double tiles = 0.0;
        double tileLength = 59.69;
        boolean temp = true;
        boolean firstLoop = true;
        boolean canRun = true;

        while (opModeIsActive() && !isStopRequested()) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("loop time (ms)", timer.milliseconds());
            timer.reset();
            telemetry.addData("X", -odometry.getPose().getY());
            telemetry.addData("Y", -odometry.getPose().getX());
            telemetry.addData("odoHeading", odometry.getPose().getHeading());
            telemetry.addData("realHeading", botHeading);
            telemetry.addData("temp", temp);
            telemetry.update();
            if (firstLoop){sleep(5000);}
            firstLoop = false;
            if (gamepad1.start) {
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                previousHeading = 0;
                processedHeading = 0;
            }

            odometry.updatePose();

            //temp = placeAndHeading(0, 20, 0, 0.4, 1, 0.5);
            //double x, double y, double idealHeading, double powerMult, double cmTol, double degTol
            //mecanum for testing
            if (gamepad1.a) {
                temp = false;
                startedAtX = -odometry.getPose().getY();
                startedAtY = -odometry.getPose().getX();
                canRun = false;
            } else {
                canRun = true;
            }
            if (temp) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX = rotX * 1.1;  // Counteract imperfect strafing
                //double botHeading = 0;
                if (Math.abs(rx) < 0.05) {rx = 0;}
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;
                motorFL.setPower(frontLeftPower);
                motorBL.setPower(backLeftPower);
                motorFR.setPower(frontRightPower);
                motorBR.setPower(backRightPower);
            } else {
                if (canRun) {
                    temp = placeAndHeading(startedAtX, startedAtY + 100, 0, 0.5, 1, 0.5);
                }
                if (!gamepad1.b) {
                    temp = false;
                }
            }
        }
    }
    /*public void moveCm(double x, double y, double powerMult, double tolerance) {
        double startX = -odometry.getPose().getY();
        double startY = odometry.getPose().getX();
        boolean done = false;
        while (!done && opModeIsActive()) {
            done = travelByNewPos(startX + x, startY + y, powerMult, tolerance);
        }
        return;
    }*/
    public boolean travelByNewPos(double x, double y, double powerMult, double tolerance) { //returns true if it's at the position.
        odometry.updatePose();
        double currentX = -odometry.getPose().getY();
        double currentY = -odometry.getPose().getX();
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
        double currentY = -odometry.getPose().getX();
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
        odometry.updatePose();
        processedHeading = newGetHeading();
        double rxConst = 6;
        double moveConst = 1;
        double currentX = -odometry.getPose().getY();
        double currentY = -odometry.getPose().getX();
        //telemetry.addData("currentX, currentY", currentX + ", " + currentY);
        double xDifference = currentX - x;
        double yDifference = currentY - y;
        double l = Math.sqrt(xDifference*xDifference + yDifference*yDifference); // diagonal error
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //x = fake joystick left left/right (strafe)
        double joyX = -1 * xDifference / l;
        //telemetry.addData("joyX", joyX);
        //y = fake joystick left up/down (move)
        double joyY = -1 * yDifference / l;
        //telemetry.addData("joyY", joyY);
        if (l < cmTol) {
            joyX = 0;
            joyY = 0;
        }
        if (l < 5) {
            moveConst = l/5;
            joyX *= moveConst;
            joyY *= moveConst;
        }
        //rx = fake joystick right left/right aka turning
        double rx = 0;
        double rotError = Math.abs(processedHeading % 360 - idealHeading);
        if (l < cmTol && rotError < degTol) {
            //if we actually don't need to do anything
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
            return true;
        }
        double sign = rotError/(processedHeading % 360 - idealHeading);
        if (rotError > 45) {rxConst = 90;}
        if (rotError < degTol) {
            rx = 0;
        } else if (sign == -1) {
            //turning left
            rx = -1 * (rxConst * 180) * rotError;
        } else if (sign == 1) {
            //turning right
            rx = (rxConst/180) * rotError;
        }
        if (rx < -1) {rx = -1;}
        if (rx > 1) {rx = 1;}
        //telemetry.addData("rX", rx);

        double rotX = joyX * Math.cos(-botHeading) - joyY * Math.sin(-botHeading);
        double rotY = joyX * Math.sin(-botHeading) + joyY * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorFL.setPower(powerMult * frontLeftPower);
        motorBL.setPower(powerMult * backLeftPower);
        motorFR.setPower(powerMult * frontRightPower);
        motorBR.setPower(powerMult * backRightPower);
        //telemetry.update();
        return false;
    }


}