package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
public class CSTeleop extends LinearOpMode {
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx motorBL;
    DcMotorEx forwardOdo;
    DcMotorEx strafeOdo;
    DcMotorEx liftEncoder;
    CRServo arm1;
    CRServo arm2;
    Servo claw1;
    Servo claw2;
    Servo lss1;
    Servo lss2;
    DcMotorEx lsm1;
    DcMotorEx lsm2;
    ServoImplEx wrist;
    DcMotorEx lift1;
    DcMotorEx lift2;

    double liftInitial;
    double ticksPerRotation;
    double liftPos;
    double liftIdealPos = 0;
    boolean liftHappyPlace = true;
    double armUpPos;
    double armDownPos;
    boolean canDriveManually = true;
    boolean canUseClawManually = true;
    boolean canDoEndgame = false;
    private IMU imu;
    double lss1UpPos = 0.5;
    double lss2UpPos = 0.5;
    boolean useLeadScrews = false;
    boolean lsStateCanChange = true;
    boolean clawStateCanChange = true;
    double claw1open;
    double claw1close;
    double claw2open;
    double claw2close;
    boolean doAbsHeading = false;
    double idealAbsHeading = 0.0;
    double turningConst = 0.5;

    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        forwardOdo = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        strafeOdo = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        arm1 = hardwareMap.get(CRServo.class, "arm1");
        arm2 = hardwareMap.get(CRServo.class, "arm2");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        lss1 = hardwareMap.get(Servo.class, "lss1");
        lss2 = hardwareMap.get(Servo.class, "lss2");
        lsm1 = hardwareMap.get(DcMotorEx.class, "lsm1");
        lsm2 = hardwareMap.get(DcMotorEx.class, "lsm2");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        ticksPerRotation = liftEncoder.getMotorType().getTicksPerRev();
        liftInitial = liftEncoder.getCurrentPosition()/ticksPerRotation;
        liftPos = (liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial;

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();
            //claw manual pickup
            if (canUseClawManually) {
                if (gamepad1.right_trigger > 0.05) {
                    //close both
                    claw1.setPosition(claw1close);
                    claw2.setPosition(claw2close);
                }
                if (gamepad1.left_trigger > 0.05) {
                    //if first one is open, open second, otherwise open first
                    if (claw1.getPosition() == claw1open) {
                        claw2.setPosition(claw2open);
                    } else {
                        claw1.setPosition(claw1open);
                    }
                }
            }
            //manual scoring-- lift/arm

            //auto pickup code

            //auto score code

            //absolute heading buttons (x/y/a/b)
            if (gamepad1.x) { //270 degrees = 3pi/2 radians
                idealAbsHeading = Math.PI * 1.5;
            } else if (gamepad1.y) { //0 degrees = 0 radians
                idealAbsHeading = 0.0;
            } else if (gamepad1.a) { //180 degrees = pi radians
                idealAbsHeading = Math.PI;
            } else if (gamepad1.b) { //90 degrees = pi/2 radians
                idealAbsHeading = Math.PI/2;
            }

            //mecanum drive code
            if (canDriveManually) {
                double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                if (Math.abs(rx) < 0.05) {rx = 0;}
                //really hope the math here works
                if (doAbsHeading) { //change rx to something that will accomplish our goal
                    double error = Math.abs((botHeading - idealAbsHeading)%360);
                    double sign = 0;
                    if (error < 0.03) {
                        rx = 0;
                        doAbsHeading = false;
                    } else {
                        if (idealAbsHeading == 0) {
                            if ((botHeading <= 2*Math.PI && botHeading >= Math.PI) || (botHeading >= -Math.PI && botHeading <= 0)) {
                                sign = 1; // turn right
                            } else {
                                sign = -1; //turn left
                            }
                        } else if (idealAbsHeading == Math.PI / 2) {
                            if ((3*Math.PI/2 <= botHeading && botHeading <= 2*Math.PI) || (-Math.PI/2 <= botHeading && Math.PI/2 >= botHeading)) {
                                sign = 1; //turn right
                            } else {
                                sign = -1; //turn left
                            }
                        } else if (idealAbsHeading == Math.PI) {
                            if ((botHeading <= 2*Math.PI && botHeading >= Math.PI) || (botHeading >= -Math.PI && botHeading <= 0)) {
                                sign = -1; // turn left
                            } else {
                                sign = 1; //turn right
                            }
                        } else if (idealAbsHeading == 3 * Math.PI / 2) {
                            if ((3*Math.PI/2 <= botHeading && botHeading <= 2*Math.PI) || (-Math.PI/2 <= botHeading && Math.PI/2 >= botHeading)) {
                                sign = -1; //turn left
                            } else {
                                sign = 1; //turn right
                            }
                        }
                        rx = error*turningConst;
                        if (rx < 0.15) {rx = 0.15;}
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

            //endgame code
            if (canDoEndgame) {
                //lead screw code
                if (gamepad1.start) {
                    lss1.setPosition(lss1UpPos);
                    lss2.setPosition(lss2UpPos);
                }
                if (gamepad1.guide && lsStateCanChange) {
                    useLeadScrews = !useLeadScrews;
                    lsStateCanChange = false;
                }
                if (!gamepad1.guide) {
                    lsStateCanChange = true;
                }
                if (useLeadScrews) {
                    //extend them to safe extension position
                } else {
                    //put them down to 0 (will work regardless of whether or not we're raising the bot)
                }
                //drone launcher code (not currently on bot)
            }
        }
    }
}
