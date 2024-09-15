package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//next import line is completely made up

import com.qualcomm.robotcore.hardware.ServoImplEx; // for Axon


@TeleOp
//still needs arm, slides, wrist, etc. code, as well as servo positions. Lift motors need to be set to hardwareMap.get().
public class RI3DTeleopID extends LinearOpMode {
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx motorBL;
    Servo arm;
    Servo claw;
    ServoImplEx wrist;
    DcMotorEx lift1;
    DcMotorEx lift2;
    private IMU imu;
    double armDownPos;
    double armUpPos;
    double armCurrentPos;
    double clawOpenPos = 0.745;
    double clawClosePos = 0.625;
    double wristDownPos;
    double previousHeading = 0;
    double processedHeading = 0;

    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //imu.resetYaw();

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandBackwardOdo");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");


        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        previousHeading = newGetHeading();
        processedHeading = previousHeading;

        waitForStart();

        arm.setPosition(armDownPos);
        claw.setPosition(clawOpenPos);
        wrist.setPosition(wristDownPos);

        double offset = 0;
        double lastHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        boolean hasBeenWeird = false;

        while (opModeIsActive()) {
            telemetry.addData("hasBeenWeird", hasBeenWeird);
            telemetry.addData("botHeading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("newHeading", botHeading);
            telemetry.addData("gamepad1.start", gamepad1.start);
            telemetry.update();

            //imu code
            if (gamepad1.start) {
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                offset = 0;
            }
            botHeading = newHeading(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), offset);
            /*if (Double.isNaN(botHeading)) {
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
            }*/

            //claw
            if (gamepad1.left_trigger > 0.05) {
                claw.setPosition(clawOpenPos);
            } else if (gamepad1.right_trigger > 0.05) {
                claw.setPosition(clawClosePos);
            }

            //arm code

            //slides code

            //mecanum
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
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
}
