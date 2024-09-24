package org.firstinspires.ftc.teamcode.intothedeep.RI3D;

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
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
//still needs arm, wrist, etc. code, as well as servo positions and lift constants. Lift motors and encoder need to be set to hardwareMap.get().
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
    DcMotorEx liftEnc;
    private IMU imu;

    //servo positions:
    //arm
    double armDownPos = 0.765; //0.945 for original straight line claw, .915 for something else
    double armStartingPos = 0.25; //0.25 for original straight line claw
    double armBucketPos = 0.4; //0.4 for original straight line claw, 0.4 for something else
    double armCurrentPos;
    //claw
    double clawOpenPos = 0.76; //0.745 for others
    double clawClosePos = 0.96; //0.625 for others
    double clawCurrentPos;
    //wrist
    double wristDownPos = 0.4; //0.5 for original straight line claw, .485 for something else
    double wristStartingPos = 0.915; //0.915 for original straight line claw, .915 for something else
    double wristBucketPos = 0.345; //0.345  for original straight line claw, .345 for something
    double wristCurrentPos;

    //imu variables
    double previousHeading = 0;
    double processedHeading = 0;
    ElapsedTime absHeadingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean absHeading;
    boolean wasJustAbsHeading;
    double imuRadians;
    double idealAbsHeading;
    double turningConst = 0.575;
    double error;

    //lift variables
    double liftInitial;
    double liftPos;
    double ticksPerRotation;
    double liftIdealPos = 0.0;
    double liftHighBasket = 0.0;
    double liftLowBasket = 0.205;
    double liftHighChamber = 0.30;//0.283 was too low, 0.295 was way too high
    double liftLowChamber = 0.155;//0.128 was too low
    double liftTolerance = 0.02;
    double kpLift = 10;
    double liftStallConst = 1.35; //was 1.3
    double liftMaxHeight = 0.31;
    double liftPower;
    double liftStallPower;

    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //imu.resetYaw();

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEnc");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        liftEnc = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEnc");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);
        ticksPerRotation = liftEnc.getMotorType().getTicksPerRev();

        previousHeading = newGetHeading();
        processedHeading = previousHeading;

        liftInitial = liftEnc.getCurrentPosition()/ticksPerRotation;
        liftPos = liftEnc.getCurrentPosition()/ticksPerRotation - liftInitial;

        waitForStart();

        arm.setPosition(armDownPos);
        armCurrentPos = armDownPos;
        claw.setPosition(clawOpenPos);
        clawCurrentPos = clawOpenPos;
        wrist.setPosition(wristDownPos);
        wristCurrentPos = wristDownPos;

        double lastHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while (opModeIsActive()) {
            liftPos = liftEnc.getCurrentPosition()/ticksPerRotation - liftInitial;
            imuRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double nghIMU = (newGetHeadingUsesRadians(imuRadians)%360) * Math.PI/180;
            telemetry.addData("botHeading", imuRadians);
            telemetry.addData("liftPower", liftPower);
            telemetry.addData("liftPos", liftPos);
            telemetry.addData("((liftPower < 0 && liftPos > 0.0) || (liftPower > 0 && liftPos < liftMaxHeight))", ((liftPower < 0 && liftPos > 0.0) || (liftPower > 0 && liftPos < liftMaxHeight)));
            telemetry.addData("(liftPower < 0 && liftPos > 0.0)", (liftPower < 0 && liftPos > 0.0));
            telemetry.addData("(liftPower > 0 && liftPos < liftMaxHeight)", (liftPower > 0 && liftPos < liftMaxHeight));
            telemetry.update();

            //imu code
            if (gamepad1.start) {
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
            }
            botHeading = nghIMU;
            //claw
            if (gamepad1.left_trigger > 0.05 && clawCurrentPos != clawOpenPos) {
                claw.setPosition(clawOpenPos);
                clawCurrentPos = clawOpenPos;
            } else if (gamepad1.right_trigger > 0.05 && clawCurrentPos != clawClosePos) {
                claw.setPosition(clawClosePos);
                clawCurrentPos = clawClosePos;
            }

            //arm code - we'll figure it out later
            if (gamepad2.dpad_up) {
                wrist.setPosition(wristBucketPos);
                wristCurrentPos = wristBucketPos;
                arm.setPosition(armBucketPos);
                armCurrentPos = armBucketPos;
            } else if (gamepad2.dpad_down) {
                wrist.setPosition(wristDownPos);
                wristCurrentPos = wristDownPos;
                arm.setPosition(armDownPos);
                armCurrentPos = armDownPos;
            }

            //slides code
            if (gamepad2.y) { //higher basket
                liftIdealPos = liftHighBasket;
                wrist.setPosition(wristBucketPos);
                wristCurrentPos = wristBucketPos;
                arm.setPosition(armBucketPos);
                armCurrentPos = armBucketPos;
            } else if (gamepad2.b) { //lower basket
                liftIdealPos = liftLowBasket;
                wrist.setPosition(wristBucketPos);
                wristCurrentPos = wristBucketPos;
                arm.setPosition(armBucketPos);
                armCurrentPos = armBucketPos;
            } else if (gamepad2.x) { //higher chamber
                liftIdealPos = liftHighChamber;
                wrist.setPosition(wristDownPos);
                wristCurrentPos = wristDownPos;
                arm.setPosition(armDownPos);
                armCurrentPos = armDownPos;
            } else if (gamepad2.a) { //lower chamber
                liftIdealPos = liftLowChamber;
                wrist.setPosition(wristDownPos);
                wristCurrentPos = wristDownPos;
                arm.setPosition(armDownPos);
                armCurrentPos = armDownPos;
            }
            liftStallPower = stallPower();//liftStallConst*liftPos;
            liftPower = -gamepad2.left_stick_y;
            double liftError = Math.abs(liftIdealPos - liftPos);
            if (Math.abs(liftPower) > 0.05 && ((liftPower < 0 && liftPos > 0.0) || (liftPower > 0 && liftPos < liftMaxHeight))) { //we are using the joysticks
                lift1.setPower(liftStallPower + liftPower);
                lift2.setPower(liftStallPower + liftPower);
                liftIdealPos = liftPos;
                telemetry.addData("liftStallPower + liftPower", liftStallPower + liftPower);
            } else if (Math.abs(liftIdealPos - liftPos) < liftTolerance) { //it's in a good spot
                lift1.setPower(liftStallPower);
                lift2.setPower(liftStallPower);
                telemetry.addData("liftStallPower", liftStallPower);
            } else if (liftIdealPos > liftPos) { //it's below where it should be
                lift1.setPower(liftStallPower + liftError * kpLift);
                lift2.setPower(liftStallPower + liftError * kpLift);
                telemetry.addData("liftStallPower + liftError * kpLift", liftStallPower + liftError * kpLift);
            } else if (liftIdealPos < liftPos) { //it's above where it should be
                lift1.setPower(liftStallPower - liftError * kpLift);
                lift2.setPower(liftStallPower - liftError * kpLift);
                telemetry.addData("liftStallPower - liftError * kpLift", liftStallPower - liftError * kpLift);
            }

            //mecanum
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if (Math.abs(rx) < 0.05) {
                rx = 0;
            }
            if (gamepad1.right_stick_button) {
                if (Math.abs(gamepad1.right_stick_x) < 0.15 && Math.abs(gamepad1.right_stick_y + 1) < 0.15) { //up -> 0 ideal
                    idealAbsHeading = 0.0;
                    absHeading = true;
                } else if (Math.abs(gamepad1.right_stick_x - 1) < 0.15 && Math.abs(gamepad1.right_stick_y) < 0.15) { //3pi/2 ideal
                    idealAbsHeading = 3*Math.PI/2;
                    absHeading = true;
                } else if (Math.abs(gamepad1.right_stick_x) < 0.15 && Math.abs(gamepad1.right_stick_y - 1) < 0.15) { //down -> pi ideal
                    idealAbsHeading = Math.PI;
                    absHeading = true;
                } else if (Math.abs(gamepad1.right_stick_x + 1) < 0.15 && Math.abs(gamepad1.right_stick_y) < 0.15) { //pi/2 ideal
                    idealAbsHeading = Math.PI/2;
                    absHeading = true;
                } else { // it's not really close to any of them
                    absHeading = false;
                }
            } else {
                absHeading = false;
            }
            if (!absHeading && wasJustAbsHeading) {
                absHeadingTimer.reset();
            }
            if (absHeadingTimer.milliseconds() < 500) {
                rx = 0;
            }
            wasJustAbsHeading = absHeading;
            if (absHeading) {
                //change rx to something that will accomplish our goal
                //if (botHeading < 0) {botHeading += 2*Math.PI;}
                botHeading = nghIMU;
                while (botHeading < 0 && opModeIsActive()) {
                    botHeading += 2*Math.PI;
                }
                while (botHeading > 2*Math.PI && opModeIsActive()) {
                    botHeading -= 2*Math.PI;
                }
                error = Math.abs((botHeading - idealAbsHeading))%(2*Math.PI);
                error = Math.min(error, 2*Math.PI - error);
                double sign = 0;
                if (error < 0.03) { //was < 0.03, 0.025 was shaky
                    rx = 0;
                    //absHeading = false;
                } else {
                    if (idealAbsHeading == 0) {
                        if ((botHeading <= 2*Math.PI && botHeading >= Math.PI) || (botHeading >= -Math.PI && botHeading <= 0)) {
                            sign = -1; // turn right
                        } else {
                            sign = 1; //turn left
                        }
                    } else if (idealAbsHeading == Math.PI / 2) {
                        if ((3*Math.PI/2 <= botHeading && botHeading <= 2*Math.PI) || (-Math.PI/2 <= botHeading && Math.PI/2 >= botHeading)) {
                            sign = -1; //turn right
                        } else {
                            sign = 1; //turn left
                        }
                    } else if (idealAbsHeading == Math.PI) {
                        if ((botHeading <= 2*Math.PI && botHeading >= Math.PI) || (botHeading >= -Math.PI && botHeading <= 0)) {
                            sign = 1; // turn left
                        } else {
                            sign = -1; //turn right
                        }
                    } else if (idealAbsHeading == 3 * Math.PI / 2) {
                        if ((3*Math.PI/2 <= botHeading && botHeading <= 2*Math.PI) || (-Math.PI/2 <= botHeading && Math.PI/2 >= botHeading)) {
                            sign = 1; //turn left
                        } else {
                            sign = -1; //turn right
                        }
                    }
                    //rx = error*turningConst; //old function with turningConst of 0.4
                    rx = turningConst*(error - Math.PI/2) + 1;
                    if (rx < 0.15) {rx = 0.15;} //was 0.2
                    if (rx > 1) {rx = 1;}
                    rx *= sign;
                }
            }
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
    public double stallPower() {
        if (liftPos < 0.015) {return 0;}

        return Math.min(1.21154*liftPos + 0.11, 0.35);
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
    public double newGetHeadingUsesRadians(double radianHeading) {
        double currentHeading = radianHeading * (180/Math.PI);
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
