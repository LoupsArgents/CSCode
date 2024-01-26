package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import org.opencv.core.Point;

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
    boolean doAutoPickup = false;
    double multiplierFR = 1.0;
    double multiplierBL = 1.0;
    double multiplierFL = 1.0;
    double multiplierBR = 1.0;
    RevColorSensorV3 clawLeftSensor;
    RevColorSensorV3 clawRightSensor;
    RevColorSensorV3 backdropDetector;
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
    ServoImplEx arm1;
    ServoImplEx arm2;
    Servo droneRelease;
    double arm1ScoringPos = 0.2675;
    double armAlmostUp = 0.37;
    double armAlmostDown = 0.8;
    double arm1DownPos = 1.0; //was 0.97
    Servo clawUp;
    Servo clawDown;
    Servo lss1;
    Servo lss2;
    DcMotorEx lsm1;
    DcMotorEx lsm2;
    ServoImplEx wrist;
    DcMotorEx lift1;
    DcMotorEx lift2;
    boolean canChangeLiftLevel = true;

    double liftInitial;
    double ticksPerRotation;
    double liftPos;
    double liftIdealPos = 0;
    boolean liftHappyPlace = true;
    //double armUpPos = 173; //was 333
    //double armDownPos = 0; //was 160
    double armVerticalPos = 216.0;
    double armPhase = 0;
    double armIdealPosition = 0;
    double armCurrentPosition;
    double armInitial;
    boolean canDriveManually = true;
    boolean canUseClawManually = true;
    boolean canDoEndgame = false;
    private IMU imu;
    double lss1UpPos = 0.575;
    double lss2UpPos = 0.395;
    double lss1DownPos = 0.575 - 0.125;
    double lss2DownPos = 0.395 + 0.125;
    boolean useLeadScrews = false;
    boolean leadScrewsDownEnd = false;
    boolean lsStateCanChange = true;
    boolean clawStateCanChange = true;
    double clawUpopen = 0.51;
    double clawUpclose = 0.355;
    double clawDownopen = 0.58;
    double clawDownclose = 0.47;
    boolean doAbsHeading = false;
    double idealAbsHeading = 0.0;
    double turningConst = 0.575;
    double wristDownPos = 0.135;
    double wristAlmostDown = 0.15;//for flipping the arm up
    double wristStraightUp = 0.45;
    double wristTuckedIn = 0.735;
    double wristScoringPos = 0.545; //was 0.54
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
    double camBarCurrentPos;
    double camInit;
    ServoImplEx cameraBar;
    double camOutOfWay = 0.36; //pointing straight out
    double camUsePos = 0.655;
    double camTuckedIn = 0.9575;
    double wristCurrentPos;
    double wristInit;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double currentTime;
    private double lastTime;
    private ElapsedTime armTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime wristTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime camBarTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double armCurrentTime;
    private double armLastTime;
    boolean leadScrewsManual = false;
    double armSetTo = 0.0;
    double wristSetTo = wristDownPos;
    double camSetTo = camTuckedIn;
    double clawUpSetTo = clawUpopen;
    double clawDownSetTo = clawDownopen;
    boolean canUseSlides = true;
    boolean isJoysticking = false;
    Point pixelPos;
    boolean hasEverSeenPixel = false;
    double pixelRowChange = 0.02;
    double pixelRow = -1;
    double secondPixelChange = 0.0125;
    //8th row (first row is 0) is 0.217516152
    //1st row (first row is 0)
    double baseBoardHeightAmt = 0.02; //good for row 0
    double stacksLevel = 0; //0 is pixels 1 and 2, 1 is pixels 2 and 3, 2 is pixels 3 and 4, 4 is pixels 4 and 5
    boolean stacksLevelCanChange = true;
    boolean phase4JustStarted = false;
    boolean doStacks = false;
    double wristStackIdeal = wristDownPos;
    boolean doAutoBoardDistance = false;
    boolean endgameCanChange = true;
    double droneInitial = 0.72; //the position we want the drone launcher servo to be at when it's not trying to launch the drone
    double droneFire = 0; //the position for the drone launcher servo that will launch the drone
    double lss2Launch = 0.4625; //the position we want for the left lead screw to be at when the drone is launching, down is 0.52, old was 0.55
    //0.46 works, 0.465 is a bit too high
    double lss2SetTo = lss2DownPos;

    //stacks positions: top level (pixels 4 and 5) arm is 0.905, wrist is 0.11
    //pixels 3 and 4 arm is 0.92, wrist is 0.12
    //pixels 2 and 3 arm is 0.945, wrist is 0.125
    //pixels 1 and 2 are normal arm/claw levels (they're on the ground)

    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        AnalogInput armAna = hardwareMap.get(AnalogInput.class, "armAna");
        armCurrentPosition = armAna.getVoltage() / 3.3 * 360;
        armInitial = armCurrentPosition;
        AnalogInput wristAna = hardwareMap.get(AnalogInput.class, "wriatAna");
        wristCurrentPos = armAna.getVoltage() / 3.3 * 360;
        wristInit = wristCurrentPos;
        AnalogInput camAna = hardwareMap.get(AnalogInput.class, "camAna");
        camBarCurrentPos = armAna.getVoltage() / 3.3 * 360;
        camInit = wristCurrentPos;

        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardEncoder");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandForwardOdo");
        arm1 = hardwareMap.get(ServoImplEx.class, "arm3"); //this is the one that DOES have an encoder
        arm2 = hardwareMap.get(ServoImplEx.class, "arm5"); //this is the one that DOES NOT have an encoder
        clawUp = hardwareMap.get(Servo.class, "claw0");
        clawDown = hardwareMap.get(Servo.class, "claw2");
        lss1 = hardwareMap.get(Servo.class, "leftLeadScrewServo");
        lss2 = hardwareMap.get(Servo.class, "rightLeadScrewServo");
        lsm1 = hardwareMap.get(DcMotorEx.class, "leadScrewRight");
        lsm2 = hardwareMap.get(DcMotorEx.class, "leadScrewLeft");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        cameraBar = hardwareMap.get(ServoImplEx.class, "frontCamera");
        lift1 = hardwareMap.get(DcMotorEx.class, "slideMotorL");
        lift2 = hardwareMap.get(DcMotorEx.class, "slideMotorR");

        droneRelease = hardwareMap.get(Servo.class, "droneRelease");

        motorFLenc = new MotorEx(hardwareMap, "motorFLandStrafeOdo");
        motorFRenc = new MotorEx(hardwareMap, "motorFRandForwardEncoder"); //also has right odometer
        motorBLenc = new MotorEx(hardwareMap, "motorBLandForwardOdo");
        motorBRenc = new MotorEx(hardwareMap, "motorBRandLiftEncoder");

        clawLeftSensor = hardwareMap.get(RevColorSensorV3.class, "clawLeft");
        clawRightSensor = hardwareMap.get(RevColorSensorV3.class, "clawRight");
        backdropDetector = hardwareMap.get(RevColorSensorV3.class, "backdropDetector");

        backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        processor = new EverythingProcessor();
        processor.setMode(EverythingProcessor.ProcessorMode.PIXEL);
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
        portal.resumeStreaming();

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        clawUp.setPosition(clawUpopen);
        clawDown.setPosition(clawDownopen);

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

        cameraBar.setPosition(camTuckedIn);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        sleep(1000);
        arm1.setPosition(arm1DownPos);
        armSetTo = arm1DownPos;

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        activateFrontCamera();

        timer.reset();
        armTimer.reset();

        while (opModeIsActive()) {
            updateAnalogs(armAna, camAna, wristAna);
            double botHeading = (newGetHeading()%360) * Math.PI/180;
            double backDistCM = backdropDetector.getDistance(DistanceUnit.CM);
            //double botHeading = Math.abs((newGetHeading()%360) * Math.PI/180);//imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //telemetry.addData("oldHeadingWay", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            currentTime = timer.milliseconds();
            armCurrentTime = armTimer.milliseconds();
            telemetry.addData("loop time, ms", currentTime);
            telemetry.addData("arm time, ms", armCurrentTime);
            telemetry.addData("backdropDistance (cm)", backDistCM);
            telemetry.addData("gamepad1.left_trigger", gamepad1.left_trigger);
            telemetry.addData("liftPos", liftPos);
            telemetry.addData("liftIdealPos", liftIdealPos);
            telemetry.addData("pixelRow", pixelRow);
            telemetry.addData("secondPixelChange", secondPixelChange);
            telemetry.addData("canChangeLiftLevel", canChangeLiftLevel);
            telemetry.addData("-gamepad2.left_stick_y", -gamepad2.left_stick_y);
            timer.reset();
            telemetry.addData("armAna", armCurrentPosition);
            telemetry.addData("wristAna", wristCurrentPos);
            telemetry.addData("camAna", camBarCurrentPos);
            telemetry.addData("botHeading", botHeading);
            //telemetry.addData("error", error);
            //telemetry.addData("idealAbsHeading", idealAbsHeading);
            //telemetry.addData("processedError", Math.min(error, 2*Math.PI - error));
            telemetry.addData("doAutoScore", doAutoScore);
            telemetry.addData("armPhase", armPhase);
            telemetry.addData("inEndgame", canDoEndgame);

            while (botHeading < 0 && opModeIsActive()) {
                botHeading += 2*Math.PI;
            }
            while (botHeading > 2*Math.PI && opModeIsActive()) {
                botHeading -= 2*Math.PI;
            }

            //safety
            if (armSetTo == arm1DownPos) {
                liftIdealPos = 0.0;
                pixelRow = -1;
            }

            //manual camera bar code
            if (gamepad2.left_bumper) {
                cameraBar.setPosition(camTuckedIn);
            } else if (gamepad2.right_bumper) {
                cameraBar.setPosition(camOutOfWay);
            }
            //arm manual code
            if (Math.abs(cameraBar.getPosition() - camTuckedIn) < 0.05 || Math.abs(cameraBar.getPosition() - camOutOfWay) < 0.05) { //we're allowed to move the arm
                //telemetry.addData("this", "runs");
                if (gamepad2.dpad_up) {
                    activateBackCamera();
                    armIdealPosition = arm1ScoringPos;
                    armPhase = 1;
                    armTimer.reset();
                    doStacks = false;
                }
                if (gamepad2.dpad_down && liftPos < 0.01) {
                    pixelRow = -1;
                    doStacks = false;
                    activateFrontCamera();
                    armIdealPosition = arm1DownPos;
                    armPhase = 2;
                    armTimer.reset();
                    if (clawUpSetTo != clawUpclose || clawDownSetTo != clawDownclose) {
                        clawUp.setPosition(clawUpclose);
                        clawUpSetTo = clawUpclose;
                        clawDown.setPosition(clawDownclose);
                        clawDownSetTo = clawDownclose;
                    }
                } else if (gamepad2.dpad_down) { //put everything all the way down
                    doStacks = false;
                    activateFrontCamera();
                    armIdealPosition = arm1DownPos;
                    armPhase = 2;
                    armTimer.reset();
                    liftIdealPos = 0.0;
                    if (clawUpSetTo != clawUpclose || clawDownSetTo != clawDownclose) {
                        clawUp.setPosition(clawUpclose);
                        clawUpSetTo = clawUpclose;
                        clawDown.setPosition(clawDownclose);
                        clawDownSetTo = clawDownclose;
                    }
                }
                //stacks positions: top level (pixels 4 and 5) arm is 0.935, wrist is 0.12
                //pixels 3 and 4 arm is 0.945, wrist is 0.12
                //pixels 2 and 3 arm is 0.955, wrist is 0.13
                //pixels 1 and 2 are normal arm/claw levels (they're on the ground)
                if (gamepad2.dpad_right) {
                    stacksLevel = 3;//0 is pixels 1 and 2, 1 is pixels 2 and 3, 2 is pixels 3 and 4, 3 is pixels 4 and 5
                    arm1.setPosition(0.935);
                    armSetTo = 0.935;
                    //wrist.setPosition(0.12);
                    wrist.setPosition(wristAlmostDown);
                    wristStackIdeal = 0.11;
                    wristTimer.reset();
                    doStacks = true;
                }
                if (gamepad2.dpad_left && stacksLevelCanChange) {
                    stacksLevelCanChange = false;
                    doStacks = true;
                    stacksLevel -= 1;
                    if (stacksLevel < 0) {stacksLevel = 0;}
                    if (stacksLevel == 2) {
                        arm1.setPosition(0.945);
                        armSetTo = 0.945;
                        //wrist.setPosition(0.12);
                        wristStackIdeal = 0.12;
                    } else if (stacksLevel == 1) {
                        arm1.setPosition(0.955);
                        armSetTo = 0.965; //was 0.955
                        //wrist.setPosition(0.125);
                        wristStackIdeal = 0.13;
                    } else if (stacksLevel == 0) {
                        arm1.setPosition(arm1DownPos);
                        armSetTo = arm1DownPos;
                        //wrist.setPosition(wristDownPos);
                        wristStackIdeal = wristDownPos;
                    }
                } else if (!gamepad2.dpad_left) {
                    stacksLevelCanChange = true;
                }
                if (doStacks && wristTimer.milliseconds() > 750) {
                    if (wristSetTo != wristStackIdeal) {
                        wrist.setPosition(wristStackIdeal);
                        wristSetTo = wristStackIdeal;
                    }
                }
                /*if (&& armPhase == 4 && wristTimer.milliseconds() > 500) {
                    if (!(wristSetTo == wristDownPos)) {
                        wrist.setPosition(wristDownPos);
                        wristSetTo = wristDownPos;
                    }
                    wristSetTo = wristDownPos;
                }*/
                if (!doStacks) {
                    if ((armPhase == 1) && armTimer.milliseconds() > 1000) { //was 2000, then 1500, 1100 worked
                        armPhase += 2;
                    }
                    if ((armPhase == 2) && armTimer.milliseconds() > 1000) { //was 2000
                        armPhase += 2;
                    }
                    if (armPhase == 1) {//just starting to go up
                        if (!(armSetTo == armAlmostUp)) {
                            arm1.setPosition(armAlmostUp);
                            armSetTo = armAlmostUp;
                        }
                        if (!(wristSetTo == wristAlmostDown)) {
                            wrist.setPosition(wristAlmostDown);
                            wristSetTo = wristAlmostDown;
                        }
                        if (clawUpSetTo != clawUpclose || clawDownSetTo != clawDownclose) {
                            clawUp.setPosition(clawUpclose);
                            clawUpSetTo = clawUpclose;
                            clawDown.setPosition(clawDownclose);
                            clawDownSetTo = clawDownclose;
                        }
                    } else if (armPhase == 2) { //just starting to go down
                        phase4JustStarted = true;
                        if (!(armSetTo == armAlmostDown)) {
                            arm1.setPosition(armAlmostDown);
                            armSetTo = armAlmostDown;
                        }
                        if (!(wristSetTo == wristAlmostDown)) {
                            wrist.setPosition(wristAlmostDown);
                            wristSetTo = wristAlmostDown;
                        }
                    } else if (armPhase == 3) { //rest of the way up
                        if (!(armSetTo == arm1ScoringPos)) {
                            arm1.setPosition(arm1ScoringPos);
                            armSetTo = arm1ScoringPos;
                        }
                        if (!(wristSetTo == wristScoringPos)) {
                            wrist.setPosition(wristScoringPos);
                            wristSetTo = wristScoringPos;
                        }
                    } else if (armPhase == 4) { //rest of the way down
                        if (phase4JustStarted) {
                            wristTimer.reset();
                        }
                        phase4JustStarted = false;
                        if (!(armSetTo == arm1DownPos)) {
                            arm1.setPosition(arm1DownPos);
                            armSetTo = arm1DownPos;
                        }
                    /*if (!(wristSetTo == wristDownPos)) {
                        wrist.setPosition(wristDownPos);
                        wristSetTo = wristDownPos;
                    }*/
                    }
                }
            }

            if (gamepad1.start) {
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
                previousHeading = 0;
                processedHeading = 0;
            }

            telemetry.update();
            //claw stuff
            if (gamepad1.right_bumper && armSetTo == arm1DownPos) {
                cameraBar.setPosition(camUsePos);
                lift1.setPower(0);
                lift2.setPower(0);
                clawUp.setPosition(clawUpopen);
                clawDown.setPosition(clawDownopen);
                canUseClawManually = false;
                canDriveManually = false;
                doAutoPickup = true;
                hasEverSeenPixel = false;
                camBarTimer.reset();
            }
            if (doAutoPickup && camBarTimer.milliseconds() > 1000 && (processor.getIsSeeingPixel() || clawSeesPixels() || hasEverSeenPixel)) {
                doAutoPickup = !closestStackInnerFunction(processor); //true = keep going, false = stop
                if (!doAutoPickup) {
                    clawUp.setPosition(clawUpclose);
                    clawUpSetTo = clawUpclose;
                    clawDown.setPosition(clawDownclose);
                    clawDownSetTo = clawDownclose;
                    gamepad1.rumble(500);
                }
                if (processor.getIsSeeingPixel() || clawSeesPixels()) {
                    hasEverSeenPixel = true;
                }
                if (gamepad1.back) {
                    doAutoPickup = false;
                }
                if (!doAutoPickup) {
                    canUseClawManually = true;
                    canDriveManually = true;
                    cameraBar.setPosition(camTuckedIn);
                }
            } else if (doAutoPickup && camBarTimer.milliseconds() > 1000 && !(processor.getIsSeeingPixel() || clawSeesPixels() || hasEverSeenPixel)) {
                doAutoPickup = false;
                canUseClawManually = true;
                canDriveManually = true;
                cameraBar.setPosition(camTuckedIn);
                gamepad1.rumble(100);
            }
            if (gamepad1.left_bumper) {
                doAutoBoardDistance = true;
            }
            //claw
            if (gamepad1.left_trigger < 0.1) {
                clawStateCanChange = true;
            }
            if (!doAutoScore && !doAutoPickup) {canDriveManually = true;}
            if (canUseClawManually) {
                if (gamepad1.right_trigger > 0.1) {
                    //close both
                    clawUp.setPosition(clawUpclose);
                    clawUpSetTo = clawUpclose;
                    clawDown.setPosition(clawDownclose);
                    clawDownSetTo = clawDownclose;
                }
                if (gamepad1.left_trigger > 0.1 && clawStateCanChange) {
                    clawStateCanChange = false;
                    //if first one is open, open second, otherwise open first
                    if (clawDownSetTo == clawDownopen) {
                        clawUp.setPosition(clawUpopen);
                        clawUpSetTo = clawUpopen;
                        pixelRow = -1;
                        secondPixelChange = 0;
                    } else {
                        clawDown.setPosition(clawDownopen);
                        clawDownSetTo = clawDownopen;
                        secondPixelChange = 0.015;
                        if (armSetTo == arm1ScoringPos && Math.abs(liftPos - liftIdealPos) < 0.01) {
                            liftIdealPos = baseBoardHeightAmt + pixelRow * pixelRowChange + secondPixelChange;
                        }
                    }
                }
            }

            //lift manual code
            if (liftPos > 0.01) {
                cameraBar.setPosition(camOutOfWay);
            }

            if (gamepad2.y && !canDoEndgame && !isJoysticking && canChangeLiftLevel) {
                canChangeLiftLevel = false;
                pixelRow++;
                liftIdealPos = baseBoardHeightAmt + pixelRow * pixelRowChange + secondPixelChange;
            } else if (!gamepad2.y && !gamepad2.a) {
                canChangeLiftLevel = true;
            }
            if (gamepad2.a && !canDoEndgame && !isJoysticking && canChangeLiftLevel) {
                canChangeLiftLevel = false;
                pixelRow--;
                if (pixelRow < 0) {
                    liftIdealPos = 0;
                } else {
                    liftIdealPos = baseBoardHeightAmt + pixelRow * pixelRowChange + secondPixelChange;
                }
            } else if (!gamepad2.a && !gamepad2.y) {
                canChangeLiftLevel = true;
            }
            if (canUseSlides) {
                if (liftIdealPos > 0.22) {
                    liftIdealPos = 0.22;
                }
                double liftPower = -gamepad2.left_stick_y;
                if (liftPower < 0) { //we're trying to go down
                    liftPower *= 0.25;
                }
                double liftError = liftIdealPos - liftPos;
                double liftTolerance = 0.005;
                double Kp = 50;
                if (Math.abs(liftError) < liftTolerance) {
                    liftHappyPlace = true;
                } else {
                    liftHappyPlace = false;
                }
                if (!isJoysticking && !liftHappyPlace) {
                    if (liftError < 0) { //going down
                        Kp = 30;
                    }
                    lift2.setPower(liftError*Kp);
                    lift1.setPower(-liftError*Kp);
                } else if (isJoysticking == false) { //liftHappyPlace == true
                    if (liftPos > 0.05) {
                        lift2.setPower(0.3);
                        lift1.setPower(-0.3);
                    } else if (liftPos > 0.125) {
                        lift2.setPower(0.4);
                        lift1.setPower(-0.4);
                    } else if (liftPos > 0.20) {
                        lift2.setPower(0.5);
                        lift1.setPower(-0.5);
                    }
                }
                if ((liftPower > 0.05 && liftPos < 0.22) || (liftPower < -0.05 && liftPos > 0)) {
                    isJoysticking = true;
                    liftHappyPlace = true;
                    liftIdealPos = liftPos;
                    lift2.setPower(liftPower);
                    lift1.setPower(-liftPower);
                } else {
                    isJoysticking = false;
                }
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
            } else {
                doAbsHeading = false;
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
                    while (botHeading < 0 && opModeIsActive()) {
                        botHeading += 2*Math.PI;
                    }
                    while (botHeading > 2*Math.PI && opModeIsActive()) {
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

            if (gamepad2.guide && endgameCanChange) {
                droneRelease.setPosition(droneInitial);
                canDoEndgame = !canDoEndgame;
                endgameCanChange = false;
            } else if (!gamepad2.guide) {
                endgameCanChange = true;
            }
            //endgame code
            if (canDoEndgame) {
                //drone launcher
                if (gamepad2.left_trigger > 0.05) {
                    lss2.setPosition(lss2Launch);
                    lss2SetTo = lss2Launch;
                }
                if (gamepad2.right_trigger > 0.05 && lss2SetTo == lss2Launch) {
                    droneRelease.setPosition(droneFire);
                }
                //lead screw code
                if (gamepad2.y) {
                    lss1.setPosition(lss1UpPos);
                    lss2SetTo = lss2UpPos;
                    lss2.setPosition(lss2UpPos);
                } else if (gamepad2.a) {
                    cameraBar.setPosition(camTuckedIn);
                    camSetTo = camTuckedIn;
                    lss1.setPosition(lss1DownPos);
                    lss2SetTo = lss2DownPos;
                    lss2.setPosition(lss2DownPos);
                }
                /*if (gamepad2.guide && lsStateCanChange) {
                    //useLeadScrews = !useLeadScrews;
                    if (!useLeadScrews) {
                        useLeadScrews = true;
                    } else {
                        useLeadScrews = false;
                        leadScrewsDownEnd = true;
                    }
                    lsStateCanChange = false;
                }*/
                //if (!gamepad2.guide) {
                //    lsStateCanChange = true;
                //}
                if (Math.abs(gamepad2.left_stick_y) > 0.05 || Math.abs(gamepad2.right_stick_y) > 0.05) {
                    useLeadScrews = false;
                    leadScrewsDownEnd = false;
                    leadScrewsManual = true;
                }
                if (gamepad2.x) {
                    leadScrewsManual = false;
                    useLeadScrews = true;
                }
                if (gamepad2.b) {
                    leadScrewsManual = false;
                    useLeadScrews = false;
                    leadScrewsDownEnd = true;
                }
                if (leadScrewsManual) {
                    if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                        lsm2.setPower(-0.5*gamepad2.left_stick_y);
                    } else {
                        lsm2.setPower(0);
                    }
                    if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                        lsm1.setPower(-0.5*gamepad2.right_stick_y);
                    } else {
                        lsm1.setPower(0);
                    }
                } else if (useLeadScrews) {
                    telemetry.addData("right lead screw", lsm1pos);
                    telemetry.addData("left lead screw", lsm2pos);
                    lsm1pos = (lsm1.getCurrentPosition()/ticksPerRotationLS)-lsm1init;
                    lsm2pos = (lsm2.getCurrentPosition()/ticksPerRotationLS)-lsm2init;

                    if (!leadScrewsManual) {
                        //extend them to safe extension position (1.21), do NOT let them get higher than 1.23
                        //positive is out, negative is back in
                        double tempPos = 1.25;
                        double error1 = Math.abs(lsm1pos - tempPos);
                        double error2 = Math.abs(lsm2pos - tempPos);
                        double lsm1const = 15;
                        double lsm2const = 5;
                        /*if (lsm1pos < tempPos) {
                            lsm1.setPower(error1*lsm1const);
                        } else { //doesn't run
                            lsm1.setPower(0); //we don't want it going any further
                        }
                        if (lsm2pos < tempPos) {
                            lsm2.setPower(error2*lsm2const);
                        } else { //doesn't run
                            lsm2.setPower(0); //we don't want it going any further
                        }*/
                        if (lsm1pos < tempPos) {
                            lsm1.setPower(error1*lsm1const);
                        } else { //doesn't run
                            lsm1.setPower(-0.05);
                        }
                        if (lsm2pos < tempPos) {
                            lsm2.setPower(error2*lsm2const);
                        } else { //doesn't run
                            lsm2.setPower(-0.05); //we don't want it going any further
                        }
                    }

                } else if (leadScrewsDownEnd) {
                    telemetry.addData("right lead screw", lsm1pos);
                    telemetry.addData("left lead screw", lsm2pos);
                    lsm1pos = (lsm1.getCurrentPosition()/ticksPerRotationLS)-lsm1init;
                    lsm2pos = (lsm2.getCurrentPosition()/ticksPerRotationLS)-lsm2init;
                    //put them down to 0.3 or something
                    //maybe at the end set the powers to -0. something so that the bot stays up?
                    double error1 = Math.abs(lsm1pos - 0.5);
                    double error2 = Math.abs(lsm2pos - 0.55);
                    double lsm1const = -15;
                    double lsm2const = -5;
                    if (lsm1pos > 0.5) {
                        //lsm1.setPower(error1*lsm1const);
                        lsm1.setPower(-1);
                    } else {
                        lsm1.setPower(0); //we don't want it going any further
                    }
                    if (lsm2pos > 0.5) {
                        //lsm2.setPower(error2*lsm2const);
                        lsm2.setPower(-1);
                    } else {
                        lsm2.setPower(0); //we don't want it going any further
                    }
                }
                //drone launcher code (not currently on bot)
            }
        }
    }
    public void updateAnalogs(AnalogInput armAna, AnalogInput camAna, AnalogInput wristAna) {
        //update arm position
        armCurrentPosition = (armAna.getVoltage() / 3.3 * 360) - armInitial;
        //update camera bar position
        camBarCurrentPos = (camAna.getVoltage() / 3.3 * 360) - camInit;
        //update wrist position
        wristCurrentPos = (wristAna.getVoltage() / 3.3 * 360) - wristInit;
        //update all odo encoders
        odometry.updatePose(); //I think???
        //update lift encoder
        liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);
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
    public boolean closestStackInnerFunction(EverythingProcessor processor){
        double leftSensorPos = clawLeftSensor.getDistance(DistanceUnit.INCH);
        double rightSensorPos = clawRightSensor.getDistance(DistanceUnit.INCH);
        boolean isThereAPixel = leftSensorPos < 2 || rightSensorPos < 2;
        if(!isThereAPixel){ //y threshold was 300
            //RobotLog.aa("DistanceFromCenter", String.valueOf(Math.abs(pixelPos.x - 320)));
            double power = .35;
            double multiplier = 1;
            double proportionalConstant = -.02;
            // used to be -.5, then -.3, then -.03, then -.01, then -.015, then -.03, then -0.025, then new camera
            //then realized that the lower limit should be -1 not 0
            //so that necessitated new finding of a constant
            //-.015
            pixelPos = processor.getClosestPixelPos();
            //RobotLog.aa("PixelPos", String.valueOf(pixelPos));
            if(Math.abs(pixelPos.x - 320) < 10){ //we're close enough to centered to just go straight forwards
                //  RobotLog.aa("Motors", "all the same");
                motorBL.setPower(power * multiplierBL);
                motorBR.setPower(power * multiplierBR);
                motorFL.setPower(power * multiplierFL);
                motorFR.setPower(power * multiplierFR);
            }else if(pixelPos.x < 320){ //we need to go left (reduce FR, BL)
                multiplier = 1 + (Math.abs(320 - pixelPos.x) * proportionalConstant);
                if(multiplier < -1){
                    RobotLog.aa("Status", "Hit the multiplier floor");
                    multiplier = -1; //so it doesn't start going backwards
                }
                RobotLog.aa("multiplier", String.valueOf(multiplier));
                telemetry.addData("multiplier", Double.toString(multiplier));
                telemetry.update();
                //RobotLog.aa("Going", "left");
                //RobotLog.aa("Motors", "setting FL and BR to " + (-power * multiplier));
                motorFR.setPower(power * multiplierFR);
                motorBL.setPower(power * multiplierBL);
                //we'll see what needs to get modified with multiplier
                //and if that needs to change at all
                motorFL.setPower(power * multiplierFL * multiplier);
                motorBR.setPower(power * multiplierBR * multiplier);
            }else if(pixelPos.x > 320){ //go right (reduce FL, BR)
                multiplier = 1 + (Math.abs(320 - pixelPos.x) * proportionalConstant);
                if(multiplier < -1){
                    RobotLog.aa("Status", "Hit the multiplier floor");
                    multiplier = -1; //so it doesn't start going backwards
                }
                telemetry.addData("multiplier", Double.toString(multiplier));
                telemetry.update();
                //RobotLog.aa("multiplier", String.valueOf(multiplier));
                //RobotLog.aa("Going", "right");
                //RobotLog.aa("Motors", "decreasing FR and BL to " + (-power * multiplier));
                motorFL.setPower(power * multiplierFL);
                motorBR.setPower(power * multiplierBR);
                motorFR.setPower(power * multiplierFR * multiplier);
                motorBL.setPower(power * multiplierBL * multiplier);
            }
            return false;
        }else{
            return true;
        }
    }
    public boolean placeAndHeading(double x, double y, double idealHeading, double powerMult, double cmTol, double degTol) {
        //odometry.updatePose();
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
    public boolean clawSeesPixels() {
        double leftSensorPos = clawLeftSensor.getDistance(DistanceUnit.INCH);
        double rightSensorPos = clawRightSensor.getDistance(DistanceUnit.INCH);
        return (leftSensorPos < 2 || rightSensorPos < 2);
    }
}
