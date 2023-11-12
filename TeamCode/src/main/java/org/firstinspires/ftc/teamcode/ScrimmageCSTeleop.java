package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class ScrimmageCSTeleop extends LinearOpMode {
    double multiplierFR = 1.0;
    double multiplierBL = 1.0;
    double multiplierFL = 1.0;
    double multiplierBR = 1.0;
    WebcamName webcam;
    VisionPortal portal;
    EverythingProcessor processor;
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx motorBL;
    DcMotorEx forwardOdo;
    DcMotorEx strafeOdo;
    DcMotorEx arm;
    ServoImplEx claw;
    ServoImplEx turret;
    ServoImplEx poleGuide;
    ServoImplEx v4b;
    ServoImplEx wrist;
    private IMU imu;

    int armDownPos = 0; //was -100, then 0, then 200
    int armUpPos = -800; //was -1390, then -700, then -600
    int armTestPos = -475; //was -700
    double clawOpenPos = 0.9; //claw is being super weird-- won't move at all
    double clawClosePos = 0.6; //same problem with the claw
    double turretPos = 0.525; //actually good!
    double poleGuideDownPos = 0.3; //good
    double poleGuideScoringPos = 0.6; //decent
    double v4bDownPos = .55; //correct - used to be 0.55
    double v4bUpPos = 0.55; //0.2 for back delivery, 0.45 should be parallel to ground, was 0.5, then 0.45
    double wristDownPos = 0.205; //was 0.225 (tilted too far left), 0.21 still too far left
    //double wristUpPos = 0.87; //no way to know w/o arm flipping
    int armTarget = 0;
    boolean isScoringPos = false;

    double previousHeading = 0;
    double processedHeading = 0;
    boolean poleCanChange = true;
    boolean poleIsUp = false;
    boolean armCanChange = true;
    boolean armIsUp = false;
    int armInitial;


    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //imu.resetYaw();

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        forwardOdo = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        strafeOdo = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandBackwardOdo");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        poleGuide = hardwareMap.get(ServoImplEx.class, "poleGuide");
        v4b = hardwareMap.get(ServoImplEx.class, "v4b");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        armInitial = arm.getCurrentPosition();
        arm.setTargetPosition(armInitial);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //arm.setTargetPosition(0);

        previousHeading = newGetHeading();
        processedHeading = previousHeading;

        armDownPos = armInitial;
        armUpPos += armInitial;

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        processor = new EverythingProcessor();
        processor.setMode(1);
        portal = VisionPortal.easyCreateWithDefaults(webcam, processor);
        portal.resumeStreaming();

        waitForStart();
        arm.setTargetPosition(armDownPos);
        claw.setPosition(clawOpenPos);
        wrist.setPosition(wristDownPos);
        v4b.setPosition(v4bDownPos);
        turret.setPosition(turretPos);
        poleGuide.setPosition(poleGuideDownPos);
        wrist.setPosition(wristDownPos);
        double currentPos = 0.5;
        double offset = 0;
        double lastHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //Servo servoToUse = claw;
        int armInitial = arm.getCurrentPosition();
        boolean hasBeenWeird = false;
        while (opModeIsActive()) {
            //botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = newHeading(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), offset);
            if (Double.isNaN(botHeading)) {
                offset = lastHeading;
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                hasBeenWeird = true;
            } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) == 0.0 || imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) == -0.0) {
                offset = lastHeading;
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                hasBeenWeird = true;
            } else {
                lastHeading = botHeading;
            }
            wrist.setPosition(wristDownPos);
            int armCurrent = arm.getCurrentPosition();
            telemetry.addData("hasBeenWeird", hasBeenWeird);
            telemetry.addData("botHeading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("newHeading", botHeading);
            telemetry.addData("gamepad1.start", gamepad1.start);
            telemetry.update();
            turret.setPosition(turretPos);
            if (gamepad1.left_trigger > 0.05) {
                claw.setPosition(clawOpenPos);
            } else if (gamepad1.right_trigger > 0.05) {
                claw.setPosition(clawClosePos);
            }
            if (gamepad1.left_bumper && armCanChange) {
                if (armIsUp) {
                   isScoringPos = false;
                   v4b.setPosition(v4bDownPos);
                   wrist.setPosition(wristDownPos);
                   armTarget = armDownPos;
                   arm.setTargetPosition(armTarget);
                   arm.setPower(0.5);
                   armIsUp = false;
                   armCanChange = false;
                } else {
                    v4b.setPosition(v4bUpPos);
                    armTarget = armTestPos;
                    arm.setTargetPosition(armTestPos);
                    arm.setPower(-0.5);
                    isScoringPos = true;
                    armIsUp = true;
                    armCanChange = false;
                }
            } else if (!gamepad1.left_bumper) {
                armCanChange = true;
            }
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (gamepad1.back && poleCanChange) {
                poleCanChange = false;
                if (poleIsUp) {
                    poleIsUp = false;
                    poleGuide.setPosition(poleGuideDownPos);
                } else {
                    poleIsUp = true;
                    poleGuide.setPosition(poleGuideScoringPos);
                }
            } else if (!gamepad1.back) {
                poleCanChange = true;
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                offset = 0;
            }

            //double botHeading = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI) % (2*Math.PI) - Math.PI;
            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

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

            if (gamepad1.right_bumper) {
                boolean isDone = false;
                while (!isDone) {
                    isDone = centerOnClosestStack(processor);
                    if (dpadIsPressed()) { //cancel
                        isDone = true;
                    }
                }
            }
        }
    }
    public double newHeading(double reading, double change) {
        double val = reading - change;
        if (val > Math.PI) {
            val -= (2*Math.PI);
        } else if (val < -Math.PI) {
            val += (2*Math.PI);
        }
        return val;
    }
    public boolean dpadIsPressed() {
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left;
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
    public boolean centerOnClosestStack(EverythingProcessor processor){ //current - diagonal movement
    /*
    * Comments for Robin when they're moving this to teleop:
    * You're going to need to copy over the EverythingProcessor class that lives in this file to your teleop.
    * If you want, you could also extend this so you have that class -- that would have the benefit of making sure any changes
    * that get made to this version automatically apply to the teleop version rather than needing to be manually copied over.
    * There's some camera setup in initializeHardware that you need, and also some in the runOpMode of this file.
    * I've copied it below so you don't have to find it:
    *
      webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
      processor = new EverythingProcessor();
      processor.setMode(1);
      portal = VisionPortal.easyCreateWithDefaults(webcam, processor);
      portal.resumeStreaming();
    *
    * Once you have that code, all the stuff in this function should work.
    * */
        double power = .35;
        Point pixelPos = processor.getClosestPixelPos();
        //^^^ this is the input of CV on this algorithm - telling us whether we're left/right of center and how much
        //which is why (see getClosestPixelPos() method below) it's important that these
        //coordinates include the left/right center of the detected object
        double multiplier = 1;
        if (opModeIsActive() && (/*Math.abs(pixelPos.x-320) > 10 ||*/ pixelPos.y < 340)){ //y threshold was 300
            //RobotLog.aa("DistanceFromCenter", String.valueOf(Math.abs(pixelPos.x - 320)));
            double proportionalConstant = -.025; // used to be -.5, then -.3, then -.03, then -.01, then -.015, then -.03; Desmos said -0.00344828
            pixelPos = processor.getClosestPixelPos();
            if(Math.abs(pixelPos.x - 320) < 10){ //we're close enough to centered to just go straight backwards
                RobotLog.aa("Motors", "all the same");
                motorBL.setPower(-power * multiplierBL);
                motorBR.setPower(-power * multiplierBR);
                motorFL.setPower(-power * multiplierFL);
                motorFR.setPower(-power * multiplierFR);
            }else if(pixelPos.x < 320){ //we need to go left (reduce FR, BL)
                multiplier = 1 + (Math.abs(320 - pixelPos.x) * proportionalConstant);
                if(multiplier < 0){
                    multiplier = 0; //so it doesn't start turning
                }
                RobotLog.aa("multiplier", String.valueOf(multiplier));
                telemetry.addData("multiplier", Double.toString(multiplier));
                telemetry.update();
                RobotLog.aa("Going", "left");
                RobotLog.aa("Motors", "setting FL and BR to " + (-power * multiplier));
                motorFR.setPower(-power * multiplierFR);
                motorBL.setPower(-power * multiplierBL);
                motorFL.setPower(-power * multiplierFL * multiplier);
                motorBR.setPower(-power * multiplierBR * multiplier);
            }else if(pixelPos.x > 320){ //go right (reduce FL, BR)
                multiplier = 1 + (Math.abs(320 - pixelPos.x) * proportionalConstant);
                if(multiplier < 0){
                    multiplier = 0; //so it doesn't start turning
                }
                telemetry.addData("multiplier", Double.toString(multiplier));
                telemetry.update();
                RobotLog.aa("multiplier", String.valueOf(multiplier));
                RobotLog.aa("Going", "right");
                RobotLog.aa("Motors", "decreasing FR and BL to " + (-power * multiplier));
                motorFL.setPower(-power * multiplierFL);
                motorBR.setPower(-power * multiplierBR);
                motorFR.setPower(-power * multiplierFR * multiplier);
                motorBL.setPower(-power * multiplierBL * multiplier);
            }
            return false;
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
            claw.setPosition(clawClosePos);
            return true;
        }
    }

}
