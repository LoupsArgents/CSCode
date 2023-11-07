package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

import android.graphics.Canvas;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@Autonomous

public class PPBotCSDF extends LinearOpMode {
    Blinker control_Hub;

    //strafe pod is in port 0, forward/back pod is in port 3
    IMU imu;
    WebcamName webcam;
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
    VisionPortal portal;
    EverythingProcessor processor;
    int armDownPos = 200; //was -100, then 0
    int armUpPos = -600; //was -1390, then -700
    int armSlightlyOffGroundPos;
    int armSpikeMarkPos = 0; //was -100
    double clawOpenPos = 0.9; //claw is being super weird-- won't move at all
    double clawClosePos = 0.6; //same problem with the claw
    double turretPos = 0.525; //actually good!
    double poleGuideDownPos = 0.3; //good
    double poleGuideScoringPos = 0.6; //decent
    double v4bDownPos = .55; //correct - used to be 0.55
    double v4bSlightlyUpPos = .56;
    double v4bUpPos = 0.5; //0.2 for back delivery, 0.45 should be parallel to ground
    double wristDownPos = 0.20; //was 0.225 (tilted too far left), 0.21 still too far left
    //double wristUpPos = 0.87; //no way to know w/o arm flipping
    int armTarget = 0;
    int strafeInitialTicks;
    int forwardInitialTicks;
    double previousHeading;
    double processedHeading;
    double multiplierFR = 1.0;
    double multiplierBL = 1.0;
    double multiplierFL = 1.0;
    double multiplierBR = 1.0;
    int armInitial;
    public void runOpMode(){
        initializeHardware();
        openClaw();
        //portal.stopStreaming();
        processor.setMode(1);
        waitForStart();
        centerOnClosestStack(processor);
        while(opModeIsActive()){

        }
        /*String propPosition = teamProcessor.getResult();
        telemetry.addData("Position", propPosition);
        telemetry.update();
        goStraight(.3, 16, 0.0);
        if(propPosition.equals("Right")){
            absoluteHeading(.2, -45.0);
            goStraight(.3, 3, -45.0);
            openClaw();
            sleep(500);
            goBackward(.3, 4, -45.0);
            absoluteHeading(.2, 0.0);
        }else if(propPosition.equals("Left")){
            absoluteHeading(.2, 45.0);
            goStraight(.3, 4, 45.0);
            openClaw();
            sleep(500);
            goBackward(.3, 2, 45.0);
            absoluteHeading(.2, 0.0);
        }else{
            goStraight(.3, 4);
            sleep(500);
            strafeRight(.35, 2.5, 5 ,0.0);
            sleep(500);
            goStraight(.3, 4.5, 0.0);
            openClaw();
            sleep(500);
            goBackward(.3, 4.5, 0.0);
            strafeLeft(.35, 2.5, 5, 0.0);
            goBackward(.3, 4);
        }
        goBackward(.2, 15);
        strafeRight(.4, 35, 10, 0.0);
        while(opModeIsActive()){

        }*/ //an old start of an auto I think
    }
    public void goStraight(double power, double inches, double idealHeading){
        double multiplier;
        //double processedInches = -1 * inches;
        telemetry.addData("RawPos", forwardOdo.getCurrentPosition());
        double forwardBackStart = getInchesTraveled(forwardOdo, 0); //second used to be 0 but IDK why
        telemetry.addData("Start", forwardBackStart);
        //The variable above gives the position, in inches, that the encoder was in at the start of the function.
        double forwardBackPos = getInchesTraveledFromIn(forwardOdo, forwardBackStart);
        telemetry.addData("FirstPos", forwardBackPos);
        motorFR.setPower(-power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(-power); //because of my differing definition of "front of the robot"
        double targetheading = idealHeading;
        RobotLog.aa("GoStraight", "goal heading is " + targetheading);
        while(forwardBackPos < inches && opModeIsActive()){
            telemetry.addData("Position", forwardBackPos);
            double heading = newGetHeading();
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(-power * multiplier);
                motorBR.setPower(-power * multiplier);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                motorFL.setPower(-power*multiplier);
                motorBL.setPower(-power*multiplier);
            }
            forwardBackPos = getInchesTraveledFromIn(forwardOdo, forwardBackStart);
        }
        stopMotors();
        telemetry.addData("Inches", getInchesTraveled(forwardOdo, 0));
        telemetry.update();
    }
    public void goBackward(double power, double inches, double idealHeading){
        double multiplier;
        double forwardBackStart = getInchesTraveled(forwardOdo, 0); //second was 0 but IDK why
        telemetry.addData("Start", forwardBackStart);
        //The variable above gives the position, in inches, that the encoder was in at the start of the function.
        double forwardBackPos = getInchesTraveledFromIn(forwardOdo, forwardBackStart);
        RobotLog.aa("FirstPos", String.valueOf(forwardBackPos));
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("GoBackward", "goal heading is " + targetheading);
        while(forwardBackPos > -inches && opModeIsActive()){
            double heading = newGetHeading();
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
                motorFR.setPower(power);
                motorBR.setPower(power);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                motorFR.setPower(power*multiplier);
                motorBR.setPower(power*multiplier);
                motorFL.setPower(power);
                motorBL.setPower(power);
            }
            forwardBackPos = getInchesTraveledFromIn(forwardOdo, forwardBackStart);
        }
        stopMotors();
    }
    //going to the right is positive ticks, so going to the left will be negative
    public void strafeRight(double power, double inches, double timelimit, double idealHeading){
        /*double multiplier;
        telemetry.addData("RawPos", forwardOdo.getCurrentPosition());
        double forwardBackStart = getInchesTraveled(forwardOdo, 0); //second used to be 0 but IDK why
        telemetry.addData("Start", forwardBackStart);
        double forwardBackPos = getInchesTraveled(forwardOdo, forwardBackStart);
        * */
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        double strafeStart = getInchesTraveled(strafeOdo, 0); //second was 0 but IDK why
        double strafePos = getInchesTraveledFromIn(strafeOdo, strafeStart);
        RobotLog.aa("Position", String.valueOf(strafePos));
        motorFR.setPower(power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeRight", "goal heading is " + targetheading);
        while(strafePos < inches && current-startthing < 1000*timelimit && opModeIsActive()){
            RobotLog.aa("Position", String.valueOf(strafePos));
            double heading = newGetHeading();
            if(heading-targetheading>=0){
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(-power);
                motorBL.setPower(power*multiplier);
                motorFR.setPower(power);
                motorBR.setPower(-power*multiplier);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                motorFR.setPower(power*multiplier);
                motorBR.setPower(-power);
                motorFL.setPower(-power*multiplier);
                motorBL.setPower(power);
            }
            strafePos = getInchesTraveledFromIn(strafeOdo, strafeStart);
            current = System.currentTimeMillis();
        }
        stopMotors();
    }
    public void strafeLeft(double power, double inches, double timelimit, double idealHeading){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        double strafeStart = getInchesTraveled(strafeOdo, 0); //second was 0 but IDK why
        RobotLog.aa("Start", String.valueOf(strafeStart));
        double strafePos = getInchesTraveledFromIn(strafeOdo, strafeStart);
        RobotLog.aa("Position", String.valueOf(strafePos));
        motorFR.setPower(-power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(-power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeLeft", "goal heading is " + targetheading);
        while(strafePos > -inches && current - startthing < 1000*timelimit && opModeIsActive()){
            RobotLog.aa("Position", String.valueOf(strafePos));
            double heading = newGetHeading();
            if(heading-targetheading>=0){
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(power*multiplier);
                motorBL.setPower(-power);
                motorFR.setPower(-power*multiplier);
                motorBR.setPower(power);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                motorFR.setPower(-power);
                motorBR.setPower(power*multiplier);
                motorFL.setPower(power);
                motorBL.setPower(-power*multiplier);
            }
            strafePos = getInchesTraveledFromIn(strafeOdo, strafeStart);
            current = System.currentTimeMillis();
        }
        stopMotors();
    }
    //below: versions of movement functions that will maintain your current heading
    public void goStraight(double power, double inches){
        goStraight(power, inches, newGetHeading());
    }
    public void goBackward(double power, double inches){
        goBackward(power, inches, newGetHeading());
    }
    public void strafeRight(double power, double inches, double timeLimit){
        strafeRight(power, inches, timeLimit, newGetHeading());
    }
    public void strafeLeft(double power, double inches, double timeLimit) {
        strafeLeft(power, inches, timeLimit, newGetHeading());
    }
    public void gyroTurn(double power, double degrees){ //right is negative
        if(opModeIsActive()){
            double gyroinitial = newGetHeading();
            if(degrees>0){ //turn left
                while(newGetHeading() - gyroinitial < degrees && opModeIsActive()){
                    motorFR.setPower(power);
                    motorBL.setPower(-power);
                    motorFL.setPower(-power);
                    motorBR.setPower(power);
                    telemetry.addData("Heading", newGetHeading());
                    telemetry.update();
                }
                stopMotors();
            }
            else{//turn right
                while(newGetHeading() - gyroinitial > degrees && opModeIsActive()){
                    motorFR.setPower(-power);
                    motorBL.setPower(power);
                    motorFL.setPower(power);
                    motorBR.setPower(-power);
                    telemetry.addData("Heading", newGetHeading());
                    telemetry.update();
                }
                stopMotors();
            }
        }
    }
    public void absoluteHeading(double power, double degrees){
        //As you can see, this only works if you also have the newGetHeading() and gyroTurn() functions.
        //gyroTurn() is where the loop is - where it would lock people out - so you might need
        //to copy all three functions and make changes to gyroTurn().
        //newGetHeading() should probably not cause any problems, though.
        double current = newGetHeading(); //Set the current heading to what the heading is, accounting for
        //the whole -179 -> 180 flip
        double processedCurrent = current % 360.0;//The processed version of the current heading
        //(This just removes extra rotations)
        telemetry.addData("how many to turn",Double.toString(degrees-processedCurrent));
        RobotLog.aa("Degrees", Double.toString(degrees));
        RobotLog.aa("ProcessedCurrent", Double.toString(processedCurrent));
        RobotLog.aa("AbsoluteHeadingShouldTurn", Double.toString(degrees-processedCurrent));
        telemetry.addData("power", power); //We probably don't need any of these telemetry things
        telemetry.update(); //But here they are
        gyroTurn(power,degrees-processedCurrent); //This is where the actual turning bit happens.
        //It uses gyroTurn(), which you'll probably have to adapt for teleop use.
    }
    public void closeClaw(){
        claw.setPosition(clawClosePos);
    }
    public void openClaw(){
        claw.setPosition(clawOpenPos);
    }
    public void stopMotors(){
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }
    public double getInchesTraveled(DcMotor encoder, int encoderInitial){ //the version that takes encoderInitial in ticks
        //Math time.
        //OK so there are 8192 ticks/rotation
        //35mm diameter of the omni wheel we use in our odo means 109.955742876mm circumference
        //This is 4.32896625496063 inches circumference.
        //Therefore, 1 rotation = 4.32896625496063 inches traveled.
        //There are 1892 ticks in an inch.
        //(This is rounded, but given how tiny ticks are, it would take a long time
        //for significant error to build up. A long, long time.)
        //Wait how long?
        //If it's approx. 0.368 ticks per inch how many inches does it take to build up to an inch of error?
        //5141 inches.
        //That's how many inches this rounding error takes to result in an inch of error.
        //That's 428 feet.
        //Given that this is far more feet than the robot should ever be moving, it doesn't matter.
        //At that point mechanical error (odo wheel slippage, etc.) would be a far bigger problem.
        //Well, after that wonderful digression, let's actually get some work done.
        //1892 ticks in an inch means we show position / 1892 and we're good to go.
        return (double)(encoder.getCurrentPosition() - encoderInitial) / 1892.0;
    }
    public double getInchesTraveledFromIn(DcMotor encoder, double encoderInitial){ //the version that takes encoderInitial in inches
        return ((double)((double)encoder.getCurrentPosition()) / 1892.0) - encoderInitial;
        //current amount moved in inches minus the first thing it was in inches
        //so why, exactly, is this not 0 when we start a strafe?
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
    public void initializeHardware(){
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        forwardOdo = hardwareMap.get(DcMotorEx.class, "motorFRandForwardOdo");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        strafeOdo = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandBackwardOdo");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        turret.setPosition(turretPos);
        poleGuide = hardwareMap.get(ServoImplEx.class, "poleGuide");
        v4b = hardwareMap.get(ServoImplEx.class, "v4b");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        strafeInitialTicks = strafeOdo.getCurrentPosition();
        forwardInitialTicks = forwardOdo.getCurrentPosition();
        armInitial = arm.getCurrentPosition();
        armDownPos = armInitial;
        armUpPos = armInitial-800;
        armSlightlyOffGroundPos = armInitial - 150;
        armSpikeMarkPos = armInitial - 250; //used to be -100, then -200
        arm.setTargetPosition(armDownPos); //see, driver hub? I'M DOING WHAT YOU WANT
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        processor = new EverythingProcessor();
        processor.setAlliance(1);
        portal = VisionPortal.easyCreateWithDefaults(webcam, processor);
        portal.resumeStreaming();
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); //this is down here in the hopes that it will stop driver hub from getting mad
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    public void centerOnClosestStack(EverythingProcessor processor){ //current - diagonal movement
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
        while(opModeIsActive() && (/*Math.abs(pixelPos.x-320) > 10 ||*/ pixelPos.y < 340)){ //y threshold was 300
            //RobotLog.aa("DistanceFromCenter", String.valueOf(Math.abs(pixelPos.x - 320)));
            double proportionalConstant = -.025; // used to be -.5, then -.3, then -.03, then -.01, then -.015, then -.03; Desmos said -0.00344828
            pixelPos = processor.getClosestPixelPos();
            if(Math.abs(pixelPos.x - 320) < 10){ //we're close enough to centered to just go straight backwards
                RobotLog.aa("Motors", "all the same");
                motorBL.setPower(-power * multiplierBL);
                motorBR.setPower(-power * multiplierBR);
                motorFL.setPower(-power * multiplierFL);
                motorFR.setPower(-power * multiplierFR);
                continue;
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
        }
        stopMotors();
        closeClaw();
    }
    //for the strafing version see the (now out of date) WingPixelDetection.java file
    static class EverythingProcessor implements VisionProcessor { //I had to make it static to make teleop work, and that has errors for all the telemetry statements so I commented them out
        double leftVal;
        double rightVal;
        double centerVal;
        int alliance; //1 is red, -1 is blue
        int setting; //0 is team prop, 1 is wing pixel
        int minPixelBoxArea = 1500;
        Point closestPixelPos = new Point(400, 400);
        Rect closestPixelRect;
        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Core.flip(frame, frame, -1);
            if(setting == 0){
                return doPropProcessing(frame);
            }else{
                return doWingProcessing(frame);
            }
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            //gotta override this so it doesn't get mad
        }
        public Object doPropProcessing(Mat frame){
            Mat mat = new Mat();
            Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                return frame;
            }
            Scalar lowHSV;
            Scalar highHSV;
            if(alliance == 1) {
                lowHSV = new Scalar(160, 70, 70);
                highHSV = new Scalar(180, 255, 255);
            }else{
                lowHSV = new Scalar(100, 70, 70);
                highHSV = new Scalar(120, 255, 255);
            }
            Mat thresh = new Mat();
            // Get a black and white image of red objects
            Core.inRange(mat, lowHSV, highHSV, thresh);

            mat.release();
            thresh.copyTo(frame);
            Imgproc.line(frame, new Point(180, 0), new Point(180, 500), new Scalar(100, 100, 100), 4, Imgproc.LINE_8);
            Imgproc.line(frame, new Point(460, 0), new Point(460, 500), new Scalar(100, 100, 100), 4, Imgproc.LINE_8);
            Mat left = thresh.submat(0, thresh.rows(), 0, thresh.cols()/3);
            Mat center = thresh.submat(0, thresh.rows(), thresh.cols()/3, 2*thresh.cols()/3);
            Mat right = thresh.submat(0, thresh.rows(), 2 * thresh.cols() / 3, thresh.cols());
            //ok now we have left, right, center: do pixel avgs.
            Core.extractChannel(left, left, 0);
            Core.extractChannel(right, right, 0);
            Core.extractChannel(center, center, 0);
            leftVal = Core.mean(left).val[0];
            rightVal = Core.mean(right).val[0];
            centerVal = Core.mean(center).val[0];
            // telemetry.addData("Left", leftVal);
            //telemetry.addData("Center", centerVal);
            //telemetry.addData("Right", rightVal);
            //telemetry.update();
            return left;
            //frame.release();
        }
        public Point getClosestPixelPos(){
            //what we're doing here is actually getting the center of the top edge of the bounding box
            //y is only important relatively, so making it the top edge doesn't hurt anything as long as we're consistent (which we are)
            //x, however (as you can see in centerOnClosestStack above) needs to represent the center of the pixel
            //
            return new Point(closestPixelPos.x + (closestPixelRect.width/2), closestPixelPos.y);
        }
        public Rect getClosestPixelRect(){
            return this.closestPixelRect;
        }
        public Object doWingProcessing(Mat frame){
            Mat mat = new Mat();
            Mat original = frame;
            Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                return frame;
            }
            Scalar lowPurpleHSV = new Scalar(117, 40, 20); //purple pixels
            Scalar highPurpleHSV = new Scalar(150, 255, 255); //120-150 should do for hue for starters
            Scalar lowYellowHSV = new Scalar(14, 100, 80); //changed lower brightness threshold b/c it saw the field as yellow lol
            Scalar highYellowHSV = new Scalar(28, 255, 255);
            Scalar lowGreenHSV = new Scalar(45,100,25);
            //the green filter is somehow perfect and has no problems. I wish all the other ones were like that.
            Scalar highGreenHSV = new Scalar(75, 255, 255);
            Scalar lowWhiteHSV = new Scalar(0,0,200); //last was 180 - updated 11/4/23 for PP offseason bot different camera angle
            Scalar highWhiteHSV = new Scalar(180, 20, 255);
            Mat purpleThresh = new Mat();
            Mat yellowThresh = new Mat();
            Mat greenThresh = new Mat();
            Mat whiteThresh = new Mat();
            // Get a black and white image of objects that are purple, yellow, green, or white
            Core.inRange(mat, lowPurpleHSV, highPurpleHSV, purpleThresh);
            Core.inRange(mat, lowYellowHSV, highYellowHSV, yellowThresh);
            Core.inRange(mat, lowGreenHSV, highGreenHSV, greenThresh);
            Core.inRange(mat, lowWhiteHSV, highWhiteHSV, whiteThresh);
            Mat testOutput = new Mat();
            Core.bitwise_or(purpleThresh, yellowThresh, testOutput);
            Core.bitwise_or(testOutput, greenThresh, testOutput);
            Core.bitwise_or(testOutput, whiteThresh, testOutput); //combine the black and white images into one black and white image of things that are game elements
            //well, it also includes things that are close in color to game elements, but that's not an issue.
            Mat masked = new Mat();
            //color the white portion of thresh in with color from original image
            //output into masked
            //Mat thing = new Mat(480, 640, CvType.CV_8UC3, new Scalar(140, 70, 200)); //purple was 140 70 200
            Core.bitwise_and(original, original, masked, testOutput);
            //Scalar average = Core.mean(masked, thresh);
            Mat scaledMask = new Mat();
            //scale the average saturation to 150
            //masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);
            testOutput.copyTo(frame);
            Mat cannyOutput = new Mat();
            int threshold = 170;
            Imgproc.blur(masked, masked, new Size(6, 6)); //was 5, 5
            Imgproc.Canny(masked, cannyOutput, threshold, threshold * 2); //edge detection wizardry copied from OpenCV tutorials - works great.
            Imgproc.blur(cannyOutput, cannyOutput, new Size(5, 5));
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); //contour wizardry copied from OpenCV tutorials
            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            Point[] centers = new Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                //bounding box wizardry copied from OpenCV tutorials
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }
            Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);
            //List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            //for (MatOfPoint2f poly : contoursPoly) {
            //    contoursPolyList.add(new MatOfPoint(poly.toArray()));
            //}
            Random rng = new Random(12345);
            /*for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
                Imgproc.drawContours(masked, contoursPolyList, i, color);
                if(boundRect[i].area() > minPixelBoxArea) {
                    //draw all bounding rectangles that pass the minimum notability threshold onto the image
                    Imgproc.rectangle(masked, boundRect[i].tl(), boundRect[i].br(), color, 2);
                    telemetry.addData("Rect", boundRect[i]);

                }
                //Imgproc.circle(masked, centers[i], (int) radius[i][0], color, 2);
            }*/
            //give me the bounding box with highest y???
            Rect maxRect = new Rect(0,0,10,10);
            for(Rect r : boundRect){
                Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
                if(r.area() > minPixelBoxArea){
                    Imgproc.rectangle(masked, r.tl(), r.br(), color, 2);
                    //telemetry.addData("Rect", r);
                }
                if(r.y > maxRect.y && r.area() > minPixelBoxArea){
                    //find the closest notable bounding box
                    maxRect = r;
                    Imgproc.putText(masked, Double.toString(r.area()), new Point(r.x, r.y), FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
                }
            }
            //Rect r = new Rect(320, 100, 10, 10);
            //Mat regionOfInterest = masked.submat(r);
            //Scalar s = Core.mean(regionOfInterest);
            //telemetry.addData("Color", s);
            //Imgproc.rectangle(masked, r, new Scalar(255, 255, 255));
            //^earlier code for finding average color over a small box to calibrate filters
            //RobotLog.aa("Box", maxRect.toString());
            closestPixelPos = new Point(maxRect.x, maxRect.y); //this is what actually informs our algorithm - see function below for a bit more processing
            closestPixelRect = maxRect;
            //telemetry.addData("BoundingBox", maxRect);
            //telemetry.update();
            masked.copyTo(frame); //always change back to masked.copyTo(frame) to see bounding boxes, etc.
            //i have a sinking suspicion that it is, in fact, the yellow filter causing these problems
            //IT IS the yellow filter! i'm sorry for ever doubting you, green filter. you're perfect.
            return frame;
        }
        public String getResult(){
            if(leftVal > rightVal && leftVal > centerVal){
                return "Left";
            }else if(rightVal > leftVal && rightVal > centerVal){
                return "Right";
            }else{
                return "Center";
            }
        }
        public void setAlliance(int a){ //0 is not-black HSV filter just for fun use on the board
            alliance = a;
        }
        public void setMode(int m){
            this.setting = m;
        }
    }
}
