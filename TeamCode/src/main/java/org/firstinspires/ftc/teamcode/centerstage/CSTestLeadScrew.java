package org.firstinspires.ftc.teamcode.centerstage;

import android.graphics.Canvas;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
public class CSTestLeadScrew extends LinearOpMode {
    DcMotorEx ls1;
    DcMotorEx ls2;
    double pos;
    double ticksPerRotation;
    double ls1init;
    double ls1pos;
    double ls2init;
    double ls2pos;
    public void runOpMode() {
        ls1 = hardwareMap.get(DcMotorEx.class, "leadScrewRight");
        ls2 = hardwareMap.get(DcMotorEx.class, "leadScrewLeft");
        ticksPerRotation = ls1.getMotorType().getTicksPerRev();
        ls1init = ls1.getCurrentPosition()/ticksPerRotation;
        ls1pos = (ls1.getCurrentPosition()/ticksPerRotation)-ls1init;
        ls2init = ls2.getCurrentPosition()/ticksPerRotation;
        ls2pos = (ls2.getCurrentPosition()/ticksPerRotation)-ls2init;

        waitForStart();
        while (opModeIsActive()) {
            ls1pos = (ls1.getCurrentPosition()/ticksPerRotation)-ls1init;
            ls2pos = (ls2.getCurrentPosition()/ticksPerRotation)-ls2init;
            telemetry.addData("rightPos", ls1pos);
            telemetry.addData("leftPos", ls2pos);
            telemetry.addData("gamepad1 left stick y", -gamepad1.left_stick_y);
            telemetry.addData("gamepad1 right stick y", -gamepad1.right_stick_y);
            telemetry.update();

            //positive powers go out, negative powers go back in
            //good value for ls1 (right lead screw) is 1.21643933955 (not above 1.23)
            //good value for ls2 (left lead screw) is 1.20495338 (not above 1.23)
            //probably 1.21 is good?
            if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                ls1.setPower(-0.5*gamepad1.left_stick_y);
            } else {
                ls1.setPower(0);
            }

            if (Math.abs(gamepad1.right_stick_y) > 0.05) {
                ls2.setPower(-0.5*gamepad1.right_stick_y);
            } else {
                ls2.setPower(0);
            }
        }
    }

}
