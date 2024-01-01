package org.firstinspires.ftc.teamcode;

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
public class ArmTestCS extends LinearOpMode {
    CRServo CR1;
    CRServo CR2;
    double s1p1;
    double s1p2;
    double s1current;
    boolean change1 = true;
    public void runOpMode() {
        //get our analog input from the hardwareMap
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "armAna");
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        double position = analogInput.getVoltage() / 3.3 * 360;

        CR1 = hardwareMap.get(CRServo.class, "arm3");
        CR2 = hardwareMap.get(CRServo.class, "arm5");
        //on broken scrimmagebot configurations: claw is part of wrist rotation, poleGuide is v4b
        s1p1 = 180;
        s1p2 = 180;
        s1current = 180;
        //servo1.setPosition(s1current);

        waitForStart();
        while (opModeIsActive()) {
            position = analogInput.getVoltage() / 3.3 * 360;
            //telemetry.addData("controls", "left trigger to decrease by 0.1, right trigger to decrease by 0.01, left bumper to increase by 0.1, right bumper to increase by 0.01");
            //telemetry.addData("controls part 2", "dpad up is set position 1, dpad down is set position 2");
            //telemetry.addData("controls part 3", "back button to move all to position 1, start button to move all to position 2");
            telemetry.addData("s1current (ideal)", s1current);
            telemetry.addData("s1current (actual)", position);
            telemetry.addData("gamepad1 left stick y", gamepad1.left_stick_y);
            telemetry.addData("\tservo 1 position 1", s1p1);
            telemetry.addData("\tservo 1 position 2", s1p2);
            telemetry.update();

            if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                double power = 0.2*gamepad1.left_stick_y;
                CR1.setPower(power);
                CR2.setPower(power);
            } else {
                CR1.setPower(0);
                CR2.setPower(0);
            }/* else {
                //if (s1current > 1) {s1current = 1;}
                if (s1current < 0) {s1current = 0;}
                //servo1.setPosition(s1current);
                setPosition(CR1, CR2, position, s1current);
                if (change1) {
                    if (gamepad1.left_trigger > 0.05) {
                        s1current -= 100;
                        change1 = false;
                    }
                    if (gamepad1.right_trigger > 0.05) {
                        s1current -= 10;
                        change1 = false;
                    }
                    if (gamepad1.left_bumper) {
                        s1current += 100;
                        change1 = false;
                    }
                    if (gamepad1.right_bumper) {
                        s1current += 10;
                        change1 = false;
                    }
                } else if (gamepad1.left_trigger < 0.05 && gamepad1.right_trigger < 0.05 && !gamepad1.left_bumper && !gamepad1.right_bumper) {
                    change1 = true;
                }
            }*/


            if (gamepad1.dpad_up) {
                s1p1 = s1current;
            }
            if (gamepad1.dpad_down) {
                s1p2 = s1current;
            }
        }
    }

    public void setPosition(CRServo c1, CRServo c2, double currentPos, double idealPos) {
        double p = Math.abs(currentPos - idealPos) * 0.001;
        if (p < 0.03) {p = 0.03;}
        if (p > 0.15) {p = 0.15;}
        if (idealPos > currentPos) {
            c1.setPower(-1 * p);
            c2.setPower(-1 * p);
        } else {
            c1.setPower(p);
            c2.setPower(p);
        }
    }

    public boolean dpadIsPressed() {
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left;
    }

}
