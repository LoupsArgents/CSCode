//package com.example.ftclibexamples;
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
@TeleOp
//@Disabled
public class DeadWheelsSample extends LinearOpMode {

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 28.6; //said 14.7

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = -11.321628; //used to be -2.1

    public static final double WHEEL_DIAMETER = 3.5; //used to say 2.0
    // if needed, one can add a gearing term here
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

        motorFLenc = new MotorEx(hardwareMap, "motorFLandStrafeOdo");
        motorFRenc = new MotorEx(hardwareMap, "motorFRandForwardOdo"); //also has right odometer
        motorBLenc = new MotorEx(hardwareMap, "motorBLandBackwardOdo");
        motorBRenc = new MotorEx(hardwareMap, "motorBRandLiftEncoder");

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandBackwardOdo");

        //motorFRenc.setDirection(Motor.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
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

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // control loop

            odometry.updatePose(); // update the position

            /*Translation2d tempTR = odometry.getPose().getTranslation();
            Rotation2d tempR = Rotation2d.fromDegrees(newGetHeading()%360);
            Pose2d tempPose = new Pose2d(odometry.getPose().getTranslation(), tempR);
            odometry.updatePose(tempPose);*/
            if (gamepad1.start) {
                resetAllPos();
            }
            if (gamepad1.back) {
                onlyResetHeading();
            }

            telemetry.addData("pos", odometry.getPose());
            telemetry.addData("leftOdometerEncoder", motorBL.getCurrentPosition());
            telemetry.addData("rightOdometerEncoder", motorFR.getCurrentPosition());
            telemetry.addData("centerOdometerEncoder", motorFL.getCurrentPosition());
            telemetry.update();

            //mecanum drive for testing
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

        }
    }
    public void onlyResetHeading() {
        Translation2d tempTR = odometry.getPose().getTranslation();
        motorFLenc = new MotorEx(hardwareMap, "motorFLandStrafeOdo");
        motorFRenc = new MotorEx(hardwareMap, "motorFRandForwardOdo"); //also has right odometer
        motorBLenc = new MotorEx(hardwareMap, "motorBLandBackwardOdo");
        motorBLenc.encoder.reset();
        motorFRenc.encoder.reset();
        motorFLenc.encoder.reset();
        leftOdometer = motorBLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said front left
        rightOdometer = motorFRenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said front right
        centerOdometer = motorFLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said back left
        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry.updatePose(new Pose2d(tempTR, Rotation2d.fromDegrees(0)));
    }
    public void resetAllPos() {
        /*Rotation2d tempR = Rotation2d.fromDegrees(0);
                Pose2d tempPose = new Pose2d(0, 0, tempR);
                odometry.updatePose(tempPose);*/
        motorFLenc = new MotorEx(hardwareMap, "motorFLandStrafeOdo");
        motorFRenc = new MotorEx(hardwareMap, "motorFRandForwardOdo"); //also has right odometer
        motorBLenc = new MotorEx(hardwareMap, "motorBLandBackwardOdo");
        motorBLenc.encoder.reset();
        motorFRenc.encoder.reset();
        motorFLenc.encoder.reset();
        leftOdometer = motorBLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said front left
        rightOdometer = motorFRenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said front right
        centerOdometer = motorFLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //said back left
        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
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

}