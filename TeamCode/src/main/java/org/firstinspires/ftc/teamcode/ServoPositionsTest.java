package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

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
public class ServoPositionsTest extends LinearOpMode {
    Servo servo1;
    Servo servo2;
    double s1p1;
    double s1p2;
    double s2p1;
    double s2p2;
    double s1current;
    double s2current;
    boolean change1 = true;
    boolean change2 = true;
    public void runOpMode() {


        servo1 = hardwareMap.get(Servo.class, "leftLeadScrewServo");
        servo2 = hardwareMap.get(Servo.class, "rightLeadScrewServo");
        //on broken scrimmagebot configurations: claw is part of wrist rotation, poleGuide is v4b
        s1p1 = 0.5;
        s1p2 = 0.5;
        s1current = 0.5;
        s2p1 = 0.5;
        s2p2 = 0.5;
        s2current = 0.5;
        servo1.setPosition(s1current);
        servo2.setPosition(s2current);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("controls", "left trigger to decrease by 0.1, right trigger to decrease by 0.01, left bumper to increase by 0.1, right bumper to increase by 0.01");
            telemetry.addData("controls part 2", "dpad up is set position 1, dpad down is set position 2");
            telemetry.addData("controls part 3", "back button to move all to position 1, start button to move all to position 2");
            telemetry.addData("s1current", s1current);
            telemetry.addData("\tservo 1 position 1", s1p1);
            telemetry.addData("\tservo 1 position 2", s1p2);
            telemetry.addData("s2current", s2current);
            telemetry.addData("\tservo 2 position 1", s2p1);
            telemetry.addData("\tservo 2 position 2", s2p2);
            telemetry.update();

            if (s1current > 1) {s1current = 1;}
            if (s1current < 0) {s1current = 0;}
            if (s2current > 1) {s2current = 1;}
            if (s2current < 0) {s2current = 0;}
            servo1.setPosition(s1current);
            servo2.setPosition(s2current);
            if (change1) {
                if (gamepad1.left_trigger > 0.05) {
                    s1current -= 0.1;
                    change1 = false;
                }
                if (gamepad1.right_trigger > 0.05) {
                    s1current -= 0.005;
                    change1 = false;
                }
                if (gamepad1.left_bumper) {
                    s1current += 0.1;
                    change1 = false;
                }
                if (gamepad1.right_bumper) {
                    s1current += 0.005;
                    change1 = false;
                }
            } else if (gamepad1.left_trigger < 0.05 && gamepad1.right_trigger < 0.05 && !gamepad1.left_bumper && !gamepad1.right_bumper) {
                change1 = true;
            }

            if (change2) {
                if (gamepad2.left_trigger > 0.05) {
                    s2current -= 0.1;
                    change2 = false;
                }
                if (gamepad2.right_trigger > 0.05) {
                    s2current -= 0.01;
                    change2 = false;
                }
                if (gamepad2.left_bumper) {
                    s2current += 0.1;
                    change2 = false;
                }
                if (gamepad2.right_bumper) {
                    s2current += 0.01;
                    change2 = false;
                }
            } else if (gamepad2.left_trigger < 0.05 && gamepad2.right_trigger < 0.05 && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                change2 = true;
            }

            if (gamepad1.dpad_up) {
                s1p1 = s1current;
            }
            if (gamepad1.dpad_down) {
                s1p2 = s1current;
            }
            if (gamepad2.dpad_up) {
                s2p1 = s2current;
            }
            if (gamepad2.dpad_down) {
                s2p2 = s2current;
            }
            if (gamepad1.back || gamepad2.back) {
                s1current = s1p1;
                s2current = s2p1;
            } else if (gamepad1.start || gamepad2.start) {
                s1current = s1p2;
                s2current = s2p2;
            }
        }
    }

    public boolean dpadIsPressed() {
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left;
    }

}
