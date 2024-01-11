package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ServoImplEx; // for Axon
import com.qualcomm.robotcore.hardware.PwmControl; // for Axon


@TeleOp
public class CSTeleop extends LinearOpMode {
    VisionPortal portal;
    private WebcamName backCamera;
    private WebcamName frontCamera;
    EverythingProcessor processor;
    AprilTagProcessor ATProcessor;
    public static final double TRACKWIDTH = 30.04;
    public static final double CENTER_WHEEL_OFFSET = -17.75; //was -19.943815, then -18
    public static final double WHEEL_DIAMETER = 3.5;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    private MotorEx motorFLenc, motorFRenc, motorBLenc, motorBRenc;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
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
    double armUpPos = 173; //was 333
    double armDownPos = 0; //was 160
    double armVerticalPos = 216.0;
    double armIdealPosition = 0;
    double armCurrentPosition;
    double armInitial;
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
    double turningConst = 0.575;
    double wristDownPos = 0.135;
    double wristAlmostDown = 0.15;//for flipping the arm up
    double wristStraightUp = 0.45;
    double wristTuckedIn = 0.735;
    double wristScoringPos = 0.54;
    double error = 0.0;

    double previousHeading = 0;
    double processedHeading = 0;
    double ticksPerRotationLS;
    double lsm1init;
    double lsm1pos;
    double lsm2init;
    double lsm2pos;
    boolean doAutoScore = false;
    double[] moveXYcm = new double[2];
    double originalX;
    double originalY;
    double headingForCV = 90;
    double allianceMultiplier = -1;
    double cmDistanceFromBoard = 5.0;
    double xFromFunction = 0;
    double yFromFunction = 0;
    boolean moveToDoingScore = true;
    boolean armHappy = true;
    double armPower = 0.0;
    boolean armPastVertical = false;

    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "armAna");
        armCurrentPosition = analogInput.getVoltage() / 3.3 * 360;
        armInitial = armCurrentPosition;

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardEncoder");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandForwardOdo");
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

        motorFLenc = new MotorEx(hardwareMap, "motorFLandStrafeOdo");
        motorFRenc = new MotorEx(hardwareMap, "motorFRandForwardEncoder"); //also has right odometer
        motorBLenc = new MotorEx(hardwareMap, "motorBLandForwardOdo");
        motorBRenc = new MotorEx(hardwareMap, "motorBRandLiftEncoder");

        backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        processor = new EverythingProcessor();
        ATProcessor = AprilTagProcessor.easyCreateWithDefaults();
        CameraName webcam = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(frontCamera, backCamera);
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 360))
                .addProcessor(processor)
                .addProcessor(ATProcessor)
                .enableLiveView(false)
                .build();
        if(portal.getProcessorEnabled(ATProcessor)) portal.setProcessorEnabled(ATProcessor, false);

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        leftOdometer = motorBLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = motorFRenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = motorFLenc.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry.updatePose(new Pose2d());

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

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        activateBackCamera();

        while (opModeIsActive()) {
            //double botHeading = Math.abs((newGetHeading()%360) * Math.PI/180);//imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = (newGetHeading()%360) * Math.PI/180;
            while (botHeading < 0) {
                botHeading += 2*Math.PI;
            }
            while (botHeading > 2*Math.PI) {
                botHeading -= 2*Math.PI;
            }
            armCurrentPosition = (analogInput.getVoltage() / 3.3 * 360) - armInitial;
            if (armIdealPosition == armUpPos) {
                armPower = setCRPosition(arm1, arm2, armCurrentPosition, armIdealPosition, armVerticalPos, 1, 0.2);
                /*if ((armCurrentPosition < armVerticalPos || Math.abs(armPower) > 0.01) && !armPastVertical) {
                    armPastVertical = false;
                    armPower = setCRPosition(arm1, arm2, armCurrentPosition, armIdealPosition, armVerticalPos, 1, 0.2);
                } else {
                    armPastVertical = true;
                    armPower = setCRPosition(arm1, arm2, armCurrentPosition, armIdealPosition, armIdealPosition, 0.1, 0.075);
                }*/
            } else {
                if ((armCurrentPosition > armVerticalPos || Math.abs(armPower) > 0.01) && !armPastVertical) {
                    armPastVertical = false;
                    armPower = setCRPosition(arm1, arm2, armCurrentPosition, armIdealPosition, armVerticalPos, 1, 0.175);
                } else {
                    armPastVertical = true;
                    armPower = setCRPosition(arm1, arm2, armCurrentPosition, armIdealPosition, armIdealPosition, 0.1, 0.075);
                }
            }


            //telemetry.addData("oldHeadingWay", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("armIdeal", armIdealPosition);
            telemetry.addData("armPosition", armCurrentPosition);
            telemetry.addData("botHeading", botHeading);
            //telemetry.addData("arm power", temp);
            telemetry.addData("error", error);
            telemetry.addData("idealAbsHeading", idealAbsHeading);
            telemetry.addData("processedError", Math.min(error, 2*Math.PI - error));
            telemetry.addData("doAutoScore", doAutoScore);

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
            if (gamepad1.left_bumper) {
                moveToDoingScore = false;
                doAutoScore = true;
                canUseClawManually = false;
                canDriveManually = false;
                double[] originalDistances = getAprilTagDist("Right");
                xFromFunction = originalDistances[0];
                yFromFunction = originalDistances[1];
                moveXYcm = new double[2];
                if (allianceMultiplier == -1) { //blue alliance
                    moveXYcm[1] = (2.54 * originalDistances[0]);
                    moveXYcm[0] = (-2.54 * originalDistances[1] + cmDistanceFromBoard);
                } else {
                    moveXYcm[1] = (-2.54 * originalDistances[0]);
                    moveXYcm[0] = (2.54 * originalDistances[1] - cmDistanceFromBoard);
                }
                originalX = -odometry.getPose().getY();
                originalY = odometry.getPose().getX();
                headingForCV = 90*allianceMultiplier;
                /*if (headingForCV < 360) {
                    headingForCV += 360;
                }*/
            } else {
                moveToDoingScore = true;
            }
            if (doAutoScore && moveToDoingScore) {
                telemetry.addData("it is running", "but not working");
                doAutoScore = !(placeAndHeading(originalX + moveXYcm[0], originalY + moveXYcm[1], headingForCV, 0.5, 0.5, 0.5));
                telemetry.addData("x cm", moveXYcm[0]);
                telemetry.addData("y cm", moveXYcm[1]);
                telemetry.addData("originalX", xFromFunction);
                telemetry.addData("originalY", yFromFunction);
            }
            if (!doAutoScore) {canDriveManually = true;}
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
            //lift is not yet ready to code
            //arm flip manual code:
            if (gamepad2.dpad_up) {
                armIdealPosition = armUpPos;
                armPastVertical = false;
            } else if (gamepad2.dpad_down) {
                armIdealPosition = armDownPos;
                armPastVertical = false;
            }

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
                if (doAbsHeading) {
                    //change rx to something that will accomplish our goal
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
                    if (error < 0.03) {
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
                        if (rx < 0.2) {rx = 0.2;}
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
    //returns [x, y]
    public double[] getAprilTagDist(String result){
        //IDs: 1 is blue left, 2 is blue center, 3 is blue right
        //4 is red left, 5 is red center, 6 is red right
        List<AprilTagDetection> currentDetections = ATProcessor.getDetections();
        double[] dists = new double[2];
        double frontDistAvg = 0.0;
        double[] leftCenterRightXDists = new double[3];
        for(AprilTagDetection d : currentDetections) {
            /*if (result.equals("Center")) {
                if (d.id == 2 || d.id == 5) {
                    xDist = d.ftcPose.x;
                    yDist = d.ftcPose.y;
                }
            } else if (result.equals("Left")) {
                if (d.id == 1 || d.id == 4) {
                    xDist = d.ftcPose.x;
                    yDist = d.ftcPose.y;
                }
            } else if (result.equals("Right")) {
                if (d.id == 3 || d.id == 6) {
                    xDist = d.ftcPose.x;
                    yDist = d.ftcPose.y;
                }
            }*/
            if(d.id == 1 || d.id == 4){
                leftCenterRightXDists[0] = d.ftcPose.x;
            }else if(d.id == 2 || d.id == 5){
                leftCenterRightXDists[1] = d.ftcPose.x;
            }else{
                leftCenterRightXDists[2] = d.ftcPose.x;
            }
            frontDistAvg += d.ftcPose.y;
        }
        double xAvg = 0.0;
        if(result.equals("Left")){
            //there appears to be 4 inches between tags (counting white barriers)
            //and the tags themselves are 2 inches wide (not counting white border)
            //average (if they exist) the left, the center -6, the right -12
            ArrayList<Double> valsToAverage = new ArrayList<>();
            if(leftCenterRightXDists[0] != 0.0){
                valsToAverage.add(leftCenterRightXDists[0]);
            }
            if(leftCenterRightXDists[1] != 0.0){
                valsToAverage.add(leftCenterRightXDists[1] - 6);
            }
            if(leftCenterRightXDists[2] != 0.0){
                valsToAverage.add(leftCenterRightXDists[2] - 12);
            }
            for(double d : valsToAverage){
                xAvg += d;
            }
            xAvg /= valsToAverage.size();
        }else if(result.equals("Center")){
            //average (if they exist) the left +6, the center, the right-6
            ArrayList<Double> valsToAverage = new ArrayList<>();
            if(leftCenterRightXDists[0] != 0.0){
                valsToAverage.add(leftCenterRightXDists[0] + 6);
            }
            if(leftCenterRightXDists[1] != 0.0){
                valsToAverage.add(leftCenterRightXDists[1]);
            }
            if(leftCenterRightXDists[2] != 0.0){
                valsToAverage.add(leftCenterRightXDists[2] - 6);
            }
            for(double d : valsToAverage){
                xAvg += d;
            }
            xAvg /= valsToAverage.size();
        }else if(result.equals("Right")){
            //average (if they exist) the left +12, the center +6, the right
            ArrayList<Double> valsToAverage = new ArrayList<>();
            if(leftCenterRightXDists[0] != 0.0){
                valsToAverage.add(leftCenterRightXDists[0] + 12);
            }
            if(leftCenterRightXDists[1] != 0.0){
                valsToAverage.add(leftCenterRightXDists[1] + 6);
            }
            if(leftCenterRightXDists[2] != 0.0){
                valsToAverage.add(leftCenterRightXDists[2]);
            }
            for(double d : valsToAverage){
                xAvg += d;
            }
            xAvg /= valsToAverage.size();
        }
        frontDistAvg /= currentDetections.size();
        dists[0] = xAvg;
        dists[1] = frontDistAvg;
        return dists;
    }
    public void activateFrontCamera(){
        if(portal.getCameraState() == VisionPortal.CameraState.STREAMING){
            portal.setActiveCamera(frontCamera);
            if(!portal.getProcessorEnabled(processor)) portal.setProcessorEnabled(processor, true);
            if(portal.getProcessorEnabled(ATProcessor)) portal.setProcessorEnabled(ATProcessor, false);
        }
    }
    public void activateBackCamera(){
        if(portal.getCameraState() == VisionPortal.CameraState.STREAMING){
            portal.setActiveCamera(backCamera);
            if(!portal.getProcessorEnabled(ATProcessor)) portal.setProcessorEnabled(ATProcessor, true);
            if(portal.getProcessorEnabled(processor)) portal.setProcessorEnabled(processor, false);
        }
    }
    public double setCRPosition(CRServo c1, CRServo c2, double position, double ideal, double idealStop, double constant, double limit) {
        //216 is vertical
        double errorCR = Math.abs(idealStop - position); //was ideal - position
        double crPower = errorCR * constant;
        if (crPower > limit) {crPower = limit;}
        if (!armPastVertical) {
            if (crPower < 0.01) {crPower = 0.1;}
        }
        if (position < idealStop) {
            crPower *= -1;
        }
        if (position < ideal && position <= 50) {
            wrist.setPosition(wristAlmostDown);
        } else if (position < ideal && position > 50) {
            wrist.setPosition(wristScoringPos);
        } else {
            wrist.setPosition(wristDownPos);
        }
        /*if (position < ideal && position > verticalPos) {
            crPower = -0.01;
        }*/
        if (position > ideal && position >= 50 && position < 140) {
            wrist.setPosition(wristAlmostDown);
        } else if (position > ideal && position < 50) {
            wrist.setPosition(wristDownPos);
        } else if (position >= 140) {
            wrist.setPosition(wristScoringPos);
        }
        /*if (position > ideal && position < verticalPos) {
            crPower = 0.01;
        }*/
        c1.setPower(crPower);
        c2.setPower(crPower);
        return crPower;
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
    public boolean placeAndHeading(double x, double y, double idealHeading, double powerMult, double cmTol, double degTol) {
        odometry.updatePose();
        processedHeading = newGetHeading();
        double rxConst = 6;
        double moveConst = 1;
        double currentX = -odometry.getPose().getY();
        double currentY = -odometry.getPose().getX();
        //telemetry.addData("currentX, currentY", currentX + ", " + currentY);
        double xDifference = currentX - x;
        double yDifference = currentY - y;
        double l = Math.sqrt(xDifference*xDifference + yDifference*yDifference); // diagonal error
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //x = fake joystick left left/right (strafe)
        double joyX = -1 * xDifference / l;
        //telemetry.addData("joyX", joyX);
        //y = fake joystick left up/down (move)
        double joyY = -1 * yDifference / l;
        //telemetry.addData("joyY", joyY);
        if (l < cmTol) {
            joyX = 0;
            joyY = 0;
        }
        if (l < 5) {
            moveConst = l/5;
            joyX *= moveConst;
            joyY *= moveConst;
        }
        //rx = fake joystick right left/right aka turning
        double rx = 0;
        double rotError = Math.abs(processedHeading % 360 - idealHeading);
        if (l < cmTol && rotError < degTol) {
            //if we actually don't need to do anything
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
            return true;
        }
        double sign = rotError/(processedHeading % 360 - idealHeading);
        if (rotError > 45) {rxConst = 90;}
        if (rotError < degTol) {
            rx = 0;
        } else if (sign == -1) {
            //turning left
            rx = -1 * (rxConst * 180) * rotError;
        } else if (sign == 1) {
            //turning right
            rx = (rxConst/180) * rotError;
        }
        if (rx < -1) {rx = -1;}
        if (rx > 1) {rx = 1;}
        //telemetry.addData("rX", rx);

        double rotX = joyX * Math.cos(-botHeading) - joyY * Math.sin(-botHeading);
        double rotY = joyX * Math.sin(-botHeading) + joyY * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorFL.setPower(powerMult * frontLeftPower);
        motorBL.setPower(powerMult * backLeftPower);
        motorFR.setPower(powerMult * frontRightPower);
        motorBR.setPower(powerMult * backRightPower);
        //telemetry.update();
        return false;
    }
}
