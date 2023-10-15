//package com.example.ftclibexamples;
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    @Override
    public void runOpMode() throws InterruptedException {
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

        boolean goodPos = false;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {


            goodPos = travelByNewPos(1, 1, 0.5, 0.1);
            odometry.updatePose(); // update the position
            telemetry.addData("pos", odometry.getPose());
            telemetry.addData("leftOdometerEncoder", motorBL.getCurrentPosition());
            telemetry.addData("rightOdometerEncoder", motorFR.getCurrentPosition());
            telemetry.addData("centerOdometerEncoder", motorFL.getCurrentPosition());
            telemetry.addData("goodPos", goodPos);
            telemetry.update();



        }
    }
    public boolean travelByNewPos(double x, double y, double powerMult, double tolerance) { //returns true if it's at the position.
        double currentX = odometry.getPose().getX();
        double currentY = odometry.getPose().getY();
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
            motorFL.setPower(BRFLPower * powerMult);
            motorBR.setPower(BRFLPower * powerMult);
            motorBL.setPower(BLFRPower * powerMult);
            motorFR.setPower(BLFRPower * powerMult);
            return false;
        } else {
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            return true;
        }
    }
    /*public boolean travelByAmount(double xToTravel, double yToTravel) { //returns true if it's at the position
        return true;
    }*/
    public double getAngleToTravel(double currentX, double currentY, double endX, double endY) { //returns angle from 0 to 2pi
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

}