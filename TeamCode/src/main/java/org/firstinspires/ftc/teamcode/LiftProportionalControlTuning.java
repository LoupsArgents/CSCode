package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;

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
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.Iterator;
import java.util.List;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ServoImplEx; // for Axon
import com.qualcomm.robotcore.hardware.PwmControl; // for Axon


@TeleOp
public class LiftProportionalControlTuning extends LinearOpMode {
    DcMotorEx lift2; //positive powers go up
    DcMotorEx lift1; //negative powers go up
    DcMotorEx liftEncoder;
    ServoImplEx arm1;
    ServoImplEx wrist;
    ServoImplEx cameraBar;
    double power = 0.0;
    boolean powerCanChange = true;
    double liftInitial;
    double liftPos;
    double ticksPerRotation;
    double camBarPos = 0.36;
    double wristPos = 0.545;
    double armPos = 0.07;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm1 = hardwareMap.get(ServoImplEx.class, "arm3");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        cameraBar = hardwareMap.get(ServoImplEx.class, "frontCamera");
        lift1 = hardwareMap.get(DcMotorEx.class, "slideMotorL");
        lift2 = hardwareMap.get(DcMotorEx.class, "slideMotorR");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");

        ticksPerRotation = liftEncoder.getMotorType().getTicksPerRev();
        liftInitial = liftEncoder.getCurrentPosition()/ticksPerRotation;
        liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);

        cameraBar.setPosition(camBarPos);
        sleep(500);
        arm1.setPosition(armPos);
        wrist.setPosition(wristPos);

        waitForStart();
        while (opModeIsActive()) {
            liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);
            telemetry.addData("controls", "dpad up and down move power up and down slightly, dpad right moves it up a lot and dpad left moves it down a lot");
            telemetry.addData("current power", power);
            telemetry.addData("liftPos", liftPos);
            telemetry.update();
            if (gamepad1.dpad_up && powerCanChange) {
                powerCanChange = false;
                power += 0.01;
            } else if (gamepad1.dpad_down && powerCanChange) {
                powerCanChange = false;
                power -= 0.01;
            } else if (gamepad1.dpad_right && powerCanChange) {
                powerCanChange = false;
                power += 0.1;
            } else if (gamepad1.dpad_left && powerCanChange) {
                powerCanChange = false;
                power -= 0.1;
            } else if (!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                powerCanChange = true;
            }
            lift2.setPower(power);
            lift1.setPower(-power);
        }
    }

}
