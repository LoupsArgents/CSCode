package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
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
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.Iterator;
import java.util.List;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ServoImplEx; // for Axon
import com.qualcomm.robotcore.hardware.PwmControl; // for Axon


@TeleOp
public class ScrimmageCSTeleop extends LinearOpMode {
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

    int armDownPos = 0; //was -100
    int armUpPos = -600; //was -1390, then -700
    int armTestPos = -700;
    double clawOpenPos = 0.9; //claw is being super weird-- won't move at all
    double clawClosePos = 0.6; //same problem with the claw
    double turretPos = 0.525; //actually good!
    double poleGuideDownPos = 0.3; //good
    double poleGuideScoringPos = 0.6; //decent
    double v4bDownPos = .55; //correct - used to be 0.55
    double v4bUpPos = 0.5; //0.2 for back delivery, 0.45 should be parallel to ground
    double wristDownPos = 0.20; //was 0.225 (tilted too far left), 0.21 still too far left
    //double wristUpPos = 0.87; //no way to know w/o arm flipping
    int armTarget = 0;
    boolean isScoringPos = false;

    double previousHeading = 0;
    double processedHeading = 0;
    boolean poleCanChange = true;
    boolean poleIsUp = false;


    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

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
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        previousHeading = newGetHeading();
        processedHeading = previousHeading;

        waitForStart();
        arm.setTargetPosition(armDownPos);
        claw.setPosition(clawOpenPos);
        wrist.setPosition(wristDownPos);
        v4b.setPosition(v4bDownPos);
        turret.setPosition(turretPos);
        poleGuide.setPosition(poleGuideDownPos);
        wrist.setPosition(wristDownPos);
        double currentPos = 0.5;
        //Servo servoToUse = claw;
        int armInitial = arm.getCurrentPosition();
        while (opModeIsActive()) {
            int armCurrent = arm.getCurrentPosition();
            telemetry.addData("currentArmPos", armCurrent);
            telemetry.addData("TargetArmPos", armTarget);
            telemetry.update();
            turret.setPosition(turretPos);
            if (gamepad1.left_trigger > 0.05) {
                claw.setPosition(clawOpenPos);
            } else if (gamepad1.right_trigger > 0.05) {
                claw.setPosition(clawClosePos);
            }
            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                v4b.setPosition(v4bUpPos);
                //wrist.setPosition(wristUpPos);
                armTarget = armUpPos; //was -1390
                arm.setTargetPosition(armTarget);
                arm.setPower(-0.5);
                isScoringPos = true;
            } else if (gamepad1.dpad_up) {
                v4b.setPosition(v4bUpPos);
                armTarget = armTestPos;
                arm.setTargetPosition(armTestPos);
                arm.setPower(-0.5);
                isScoringPos = true;
            } else if (gamepad1.dpad_down || !isScoringPos) {
                isScoringPos = false;
                v4b.setPosition(v4bDownPos);
                wrist.setPosition(wristDownPos);
                armTarget = armDownPos; //was -100
                arm.setTargetPosition(armTarget);
                arm.setPower(0.5);
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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

            if (gamepad1.a) {
                boolean done = false;
                while (!done) {
                    done = absoluteHeading(0, 0.65, 1);
                    if (gamepad1.right_bumper) { //cancel
                        done = true;
                    }
                }
            } else if (gamepad1.b) {
                boolean done = false;
                while (!done) {
                    done = absoluteHeading(90, 0.65, 1);
                    if (gamepad1.right_bumper) { //cancel
                        done = true;
                    }
                }
            } else if (gamepad1.x) {
                boolean done = false;
                while (!done) {
                    done = absoluteHeading(270, 0.65, 1);
                    if (gamepad1.right_bumper) { //cancel
                        done = true;
                    }
                }
            } else if (gamepad1.y) {
                boolean done = false;
                while (!done) {
                    done = absoluteHeading(180, 0.65, 1);
                    if (gamepad1.right_bumper) { //cancel
                        done = true;
                    }
                }
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
        }
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
}
