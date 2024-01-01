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
    Servo clawUp;
    Servo clawDown;
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
    double armUpPos = 333;
    double armDownPos = 162;
    boolean canDriveManually = true;
    boolean canUseClawManually = true;
    boolean canDoEndgame = false;
    private IMU imu;
    double lss1UpPos = 0.575;
    double lss2UpPos = 0.395;
    boolean useLeadScrews = false;
    boolean leadScrewsDownEnd = false;
    boolean lsStateCanChange = true;
    boolean clawStateCanChange = true;
    double clawUpopen = 0.5;
    double clawUpclose = 0.4;
    double clawDownopen = 0.58;
    double clawDownclose = 0.49;
    boolean doAbsHeading = false;
    double idealAbsHeading = 0.0;
    double turningConst = 0.46;
    double wristDownPos = 0.135;
    double wristAlmostDown = 0.15;//for flipping the arm up
    double wristStraightUp = 0.45;
    double wristTuckedIn = 0.735;
    double wristScoringPos = 0.0;
    double error = 0.0;

    double previousHeading = 0;
    double processedHeading = 0;
    double ticksPerRotationLS;
    double lsm1init;
    double lsm1pos;
    double lsm2init;
    double lsm2pos;

    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardEncoder");
        forwardOdo = hardwareMap.get(DcMotorEx.class, "motorFRandForwardEncoder");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandForwardOdo");
        //strafeOdo = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandStrafeOdo");
        arm1 = hardwareMap.get(CRServo.class, "arm3");
        arm2 = hardwareMap.get(CRServo.class, "arm5");
        clawUp = hardwareMap.get(Servo.class, "claw0");
        clawDown = hardwareMap.get(Servo.class, "claw2");
        lss1 = hardwareMap.get(Servo.class, "leftLeadScrewServo");
        lss2 = hardwareMap.get(Servo.class, "rightLeadScrewServo");
        lsm1 = hardwareMap.get(DcMotorEx.class, "leadScrewRight");
        lsm2 = hardwareMap.get(DcMotorEx.class, "leadScrewLeft");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        lift1 = hardwareMap.get(DcMotorEx.class, "slideMotorL");
        lift2 = hardwareMap.get(DcMotorEx.class, "slideMotorR");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        wrist.setPosition(wristDownPos);

        ticksPerRotation = liftEncoder.getMotorType().getTicksPerRev();
        liftInitial = liftEncoder.getCurrentPosition()/ticksPerRotation;
        liftPos = (liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial;

        previousHeading = newGetHeading();
        processedHeading = previousHeading;

        lsm1 = hardwareMap.get(DcMotorEx.class, "leadScrewRight");
        lsm2 = hardwareMap.get(DcMotorEx.class, "leadScrewLeft");
        ticksPerRotationLS = lsm1.getMotorType().getTicksPerRev();
        lsm1init = lsm1.getCurrentPosition()/ticksPerRotationLS;
        lsm1pos = (lsm1.getCurrentPosition()/ticksPerRotationLS)-lsm1init;
        lsm2init = lsm2.getCurrentPosition()/ticksPerRotationLS;
        lsm2pos = (lsm2.getCurrentPosition()/ticksPerRotationLS)-lsm2init;

        waitForStart();

        while (opModeIsActive()) {
            //double botHeading = Math.abs((newGetHeading()%360) * Math.PI/180);//imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = (newGetHeading()%360) * Math.PI/180;
            while (botHeading < 0) {
                botHeading += 2*Math.PI;
            }
            while (botHeading > 2*Math.PI) {
                botHeading -= 2*Math.PI;
            }
            telemetry.addData("oldHeadingWay", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("botHeading", botHeading);
            telemetry.addData("error", error);
            telemetry.addData("idealAbsHeading", idealAbsHeading);
            telemetry.addData("processedError", Math.min(error, 2*Math.PI - error));

            if (gamepad1.start) {
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                previousHeading = 0;
                processedHeading = 0;
            }

            telemetry.update();
            //claw manual pickup
            if (gamepad1.left_trigger < 0.05) {
                clawStateCanChange = true;
            }
            if (canUseClawManually) {
                if (gamepad1.right_trigger > 0.05) {
                    //close both
                    clawUp.setPosition(clawUpclose);
                    clawDown.setPosition(clawDownclose);
                }
                if (gamepad1.left_trigger > 0.05 && clawStateCanChange) {
                    clawStateCanChange = false;
                    //if first one is open, open second, otherwise open first
                    if (clawDown.getPosition() == clawDownopen) {
                        clawUp.setPosition(clawUpopen);
                    } else {
                        clawDown.setPosition(clawDownopen);
                    }
                }
            }
            //manual scoring-- lift/arm

            //auto pickup code

            //auto score code

            //absolute heading buttons (x/y/a/b)
            if (gamepad1.x) { //270 degrees = 3pi/2 radians
                //idealAbsHeading = Math.PI * 1.5;
                idealAbsHeading = Math.PI/2;
                doAbsHeading = true;
            } else if (gamepad1.y) { //0 degrees = 0 radians
                idealAbsHeading = 0.0;
                doAbsHeading = true;
            } else if (gamepad1.a) { //180 degrees = pi radians
                idealAbsHeading = Math.PI;
                doAbsHeading = true;
            } else if (gamepad1.b) { //90 degrees = pi/2 radians
                idealAbsHeading = Math.PI*1.5;
                doAbsHeading = true;
            }

            //mecanum drive code
            if (canDriveManually) {
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX = rotX * 1.1;  // Counteract imperfect strafing
                //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                //double botHeading = 0;
                if (Math.abs(rx) < 0.05) {rx = 0;}
                //really hope the math here works
                if (doAbsHeading) { //change rx to something that will accomplish our goal
                    //if (botHeading < 0) {botHeading += 2*Math.PI;}
                    //botHeading = Math.abs(newGetHeading() * (Math.PI/180));
                    botHeading = (newGetHeading()%360) * Math.PI/180;
                    while (botHeading < 0) {
                        botHeading += 2*Math.PI;
                    }
                    while (botHeading > 2*Math.PI) {
                        botHeading -= 2*Math.PI;
                    }
                    error = Math.abs((botHeading - idealAbsHeading))%(2*Math.PI);
                    error = Math.min(error, 2*Math.PI - error);
                    double sign = 0;
                    if (error < 0.05) {
                        rx = 0;
                        doAbsHeading = false;
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
                        if (rx < 0.3) {rx = 0.3;}
                        if (rx > 1) {rx = 1;}
                        rx *= sign;
                    }
                }

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
                    //useLeadScrews = !useLeadScrews;
                    if (!useLeadScrews) {
                        useLeadScrews = true;
                    } else {
                        useLeadScrews = false;
                        leadScrewsDownEnd = true;
                    }
                    lsStateCanChange = false;
                }
                if (!gamepad1.guide) {
                    lsStateCanChange = true;
                }
                if (useLeadScrews) {
                    //extend them to safe extension position (1.21), do NOT let them get higher than 1.23
                    //positive is out, negative is back in
                    double error1 = Math.abs(lsm1pos - 1.21);
                    double error2 = Math.abs(lsm2pos - 1.21);
                    double lsm1const = 0.5;
                    double lsm2const = 0.5;
                    if (lsm1pos < 1.21) {
                        lsm1.setPower(error1*lsm1const);
                    } else {
                        lsm1.setPower(0); //we don't want it going any further
                    }
                    if (lsm2pos < 1.21) {
                        lsm2.setPower(error2*lsm2const);
                    } else {
                        lsm2.setPower(0); //we don't want it going any further
                    }
                } else if (leadScrewsDownEnd) {
                    //put them down to 0.3 or something
                    //maybe at the end set the powers to -0. something so that the bot stays up?
                    double error1 = Math.abs(lsm1pos - 0.3);
                    double error2 = Math.abs(lsm2pos - 0.3);
                    double lsm1const = -0.5;
                    double lsm2const = -0.5;
                    if (lsm1pos > 0.3) {
                        lsm1.setPower(error1*lsm1const);
                    } else {
                        lsm1.setPower(0); //we don't want it going any further
                    }
                    if (lsm2pos > 0.3) {
                        lsm2.setPower(error2*lsm2const);
                    } else {
                        lsm2.setPower(0); //we don't want it going any further
                    }
                }
                //drone launcher code (not currently on bot)
            }
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
}
