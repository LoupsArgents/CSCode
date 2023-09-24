package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class NewPPTeleop extends LinearOpMode {
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx motorBL;
    DcMotorEx forwardOdo;
    DcMotorEx strafeOdo;
    DcMotorEx liftEncoder;
    DcMotorEx arm;
    ServoImplEx claw;
    ServoImplEx turret;
    ServoImplEx poleGuide;
    ServoImplEx v4b;
    ServoImplEx wrist;

    int armDownPos = 0; //was -100
    int armUpPos = -700; //was -1390
    double clawOpenPos = 0.9; //claw is being super weird-- won't move at all
    double clawClosePos = 0.6; //same problem with the claw
    double turretPos = 0.525; //actually good!
    double poleGuideDownPos = 0.3; //good
    double poleGuideScoringPos = 0.55; //decent
    double v4bDownPos = .5; //correct - used to be 0.55
    double v4bUpPos = 0.5; //0.2 for back delivery, 0.45 should be parallel to ground
    double wristDownPos = 0.225; //good
    double wristUpPos = 0.87; //no way to know w/o arm flipping
    int armTarget = 0;
    boolean poleGuideUse = true;


    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        forwardOdo = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        strafeOdo = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
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
            /*if (gamepad2.a) {currentPos = 0.1;}
            if (gamepad2.b) {currentPos = 0.2;}
            if (gamepad2.x) {currentPos = 0.3;}
            if (gamepad2.y) {currentPos = 0.4;}
            if (gamepad2.dpad_down) {currentPos = 0.5;}
            if (gamepad2.dpad_left) {currentPos = 0.6;}
            if (gamepad2.dpad_up) {currentPos = 0.7;}
            if (gamepad2.dpad_right) {currentPos = 0.8;}
            if (gamepad2.start) {currentPos = 0.9;}
            claw.setPosition(currentPos);*/
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
            }
            else if (gamepad1.dpad_down) {
                v4b.setPosition(v4bDownPos);
                wrist.setPosition(wristDownPos);
                armTarget = armDownPos; //was -100
                arm.setTargetPosition(armTarget);
                arm.setPower(0.5);
                if (poleGuideUse) {
                    poleGuide.setPosition(poleGuideDownPos);
                }
            }
            else if (gamepad1.dpad_up) {
                v4b.setPosition(v4bUpPos);
                wrist.setPosition(wristUpPos);
                armTarget = -1390;
                arm.setTargetPosition(armTarget);
                arm.setPower(-0.5);
                if (poleGuideUse) {
                    poleGuide.setPosition(poleGuideScoringPos);
                }
            }
            if (gamepad1.start && poleGuideUse == false) {
                poleGuideUse = true;
            }
            if (gamepad1.start && poleGuideUse == true) {
                poleGuideUse = false;
            }
            if (gamepad1.a) {
                v4b.setPosition(v4bDownPos);
            }
            else if (gamepad1.b) {
                v4b.setPosition(v4bUpPos);
            }
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
}
