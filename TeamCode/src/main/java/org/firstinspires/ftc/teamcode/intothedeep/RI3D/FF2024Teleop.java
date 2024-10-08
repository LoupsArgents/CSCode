package org.firstinspires.ftc.teamcode.intothedeep.RI3D;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
//next import line is completely made up
import com.qualcomm.hardware.lynx.LynxNackException;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ServoImplEx; // for Axon
import com.qualcomm.robotcore.hardware.PwmControl; // for Axon

import java.util.List;
@TeleOp
public class FF2024Teleop extends LinearOpMode {
    IMU imu;
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx liftEncoder;
    DcMotorEx motorBL;
    DcMotorEx intake;
    RevColorSensorV3 sensorColor;
    CRServo intakeLeft;
    int intakeStatus = 0; // 0 is off, 1 is take in, 2 is spit out
    boolean canChangeIntake = true;
    double botHeading;
    double intakeCurrent;
    long stallStartTime = -1;
    double r;
    double g;
    double b;
    double a;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandLiftEnc");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorFLandLiftEnc");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        sensorColor = hardwareMap.get(RevColorSensorV3.class,"color");
        intakeLeft = hardwareMap.get(CRServo.class,"intakeLeftServo");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double imuRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = imuRadians;
            intakeCurrent = intake.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("botHeading", botHeading);
            telemetry.addData("intakeStatus", intakeStatus);
            telemetry.addData("canChangeIntake", canChangeIntake);
            telemetry.addData("intake motor current",intakeCurrent);
            r = sensorColor.red();
            g = sensorColor.green();
            b = sensorColor.blue();
            a = sensorColor.alpha();

            telemetry.addData("r", r);
            telemetry.addData("g",g);
            telemetry.addData("b",b);
            telemetry.addData("a",a);
            telemetry.update();

            //mecanum
            botHeading = imuRadians;
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if (Math.abs(rx) < 0.05) {rx = 0;}
            //rotate the movement counter to the bot's heading
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing
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
            if (gamepad1.start) {
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                //previousHeading = 0;
                //processedHeading = 0;
            }
            if ((gamepad1.left_stick_button || gamepad1.right_stick_button) && (intakeStatus == 1 || intakeStatus == 2) && canChangeIntake) {
                intakeLeft.setPower(0.0);
                intake.setPower(0);
                intakeStatus = 0; // 0 is off, 1 is take in, 2 is spit out
                canChangeIntake = false;
            } //else
            if ((r > 100 || g > 100 || b > 100) && (intakeStatus == 1) && canChangeIntake) {
                intakeLeft.setPower(0.0);
                intake.setPower(0);
                intakeStatus = 0;
                canChangeIntake = false;
            } /*
            else if (intakeStatus == 2 && (r < 100 && g < 100 && b < 100)) {
                intakeLeft.setPower(0.0);
                intake.setPower(0);
                intakeStatus = 0;
                canChangeIntake = false;
            }
            */
            if (canChangeIntake && gamepad1.left_stick_button && intakeStatus == 0) {
                intakeLeft.setPower(0.9);
                intake.setPower(0.75);
                intakeStatus = 1;
                canChangeIntake = false;
            }
            if (canChangeIntake && (gamepad1.right_stick_button && intakeStatus == 0)) {
                intake.setPower(-0.5); // turned spit out power down from 0.9
                intakeStatus = 2;
                canChangeIntake = false;
                if (intakeCurrent > 6.5) {
                    if (stallStartTime == -1) {
                        stallStartTime = System.currentTimeMillis();
                    } else {
                        // find how long it's been stalling for
                        if (System.currentTimeMillis() - stallStartTime > 750) {
                            intake.setPower(-0.5);
                            intakeStatus = 2;
                            canChangeIntake = false;
                        }

                    }
                } else {
                    stallStartTime = -1;
                }
            }
            if (!gamepad1.left_stick_button && !gamepad1.right_stick_button) {
                canChangeIntake = true;
            }
        }
    }
}
