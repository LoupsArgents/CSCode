package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.List;

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
    AprilTagProcessor ATProcessor;
    int armDownPos = 200; //was -100, then 0
    int armUpPos = -600; //was -1390, then -700
    int armSlightlyOffGroundPos;
    int armBackdropDeliveryPos;
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
        v4b.setPosition(v4bSlightlyUpPos);
        arm.setPower(.5);
        arm.setTargetPosition(armSlightlyOffGroundPos);
        openClaw();
        //portal.stopStreaming();
        processor.setMode(EverythingProcessor.ProcessorMode.PIXEL);
        waitForStart();
        //centerOnClosestStack(processor);
        moveBackRight(.4, 20, 0.0);
        sleep(500);
        moveBackLeft(.4, 20, 0.0);

        // while(opModeIsActive()){

        //}
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
        int forwardBackStartTicks = forwardOdo.getCurrentPosition();
        telemetry.addData("StartTicks", forwardBackStartTicks);
        int forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        motorFR.setPower(-power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(-power); //because of my differing definition of "front of the robot"
        double targetheading = idealHeading;
        RobotLog.aa("GoStraight", "goal heading is " + targetheading);
        while(newInchesTraveled(forwardBackStartTicks, forwardBackCurrentTicks) < inches && opModeIsActive()){
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
            forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        }
        stopMotors();
    }
    public void goBackward(double power, double inches, double idealHeading){
        double multiplier;
        int forwardBackStartTicks = forwardOdo.getCurrentPosition();
        telemetry.addData("StartTicks", forwardBackStartTicks);
        int forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("GoBackward", "goal heading is " + targetheading);
        while(newInchesTraveled(forwardBackStartTicks, forwardBackCurrentTicks) > -inches && opModeIsActive()){
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
            forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        }
        stopMotors();
    }
    //going to the right is positive ticks, so going to the left will be negative
    public void strafeRight(double power, double inches, double timelimit, double idealHeading){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int strafeThirdTicks = strafeOdo.getCurrentPosition();
        RobotLog.aa("Initial", String.valueOf(strafeStartTicks));
        RobotLog.aa("InitialCurrent", String.valueOf(strafeCurrentTicks));
        RobotLog.aa("Third", String.valueOf(strafeThirdTicks));
        RobotLog.aa("InitialDifference", String.valueOf(strafeCurrentTicks - strafeStartTicks));
        RobotLog.aa("InitialInches", String.valueOf(newInchesTraveled(strafeStartTicks, strafeCurrentTicks)));
        motorFR.setPower(power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeRight", "goal heading is " + targetheading);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) < inches && current-startthing < 1000*timelimit && opModeIsActive()){
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
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
            current = System.currentTimeMillis();
        }
        stopMotors();
    }
    public void strafeLeft(double power, double inches, double timelimit, double idealHeading){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int strafeThirdTicks = strafeOdo.getCurrentPosition();
        RobotLog.aa("Initial", String.valueOf(strafeStartTicks));
        RobotLog.aa("InitialCurrent", String.valueOf(strafeCurrentTicks));
        RobotLog.aa("Third", String.valueOf(strafeThirdTicks));
        RobotLog.aa("InitialDifference", String.valueOf(strafeCurrentTicks - strafeStartTicks));
        RobotLog.aa("InitialInches", String.valueOf(newInchesTraveled(strafeStartTicks, strafeCurrentTicks)));
        motorFR.setPower(-power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(-power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeLeft", "goal heading is " + targetheading);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) > -inches && current - startthing < 1000*timelimit && opModeIsActive()){
            RobotLog.aa("Inches", String.valueOf(newInchesTraveled(strafeStartTicks, strafeCurrentTicks)));
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
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
            current = System.currentTimeMillis();
        }
        stopMotors();
    }
    public void moveForwardRight(double power, double inches, double idealHeading){
        //this is back left on other people's view of this bot -> FL and BR backwards
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        double multiplier;
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) < inches && opModeIsActive()){
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorFL.setPower(-power);
                motorBR.setPower(-power * multiplier);
                //so this means we need turn left...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorFL.setPower(-power * multiplier);
                motorBR.setPower(- power);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
        }
        stopMotors();
    }
    public void moveForwardLeft(double power, double inches, double idealHeading){
        //this is back right on other people's view of this bot -> FR and BL backwards
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        double multiplier;
        motorFR.setPower(-power);
        motorBL.setPower(-power);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) > -inches && opModeIsActive()){
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorBL.setPower(-power);
                motorFR.setPower(-power * multiplier);
                //so this means we need turn left...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorBL.setPower(-power * multiplier);
                motorFR.setPower(-power);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
        }
        stopMotors();
    }
    public void moveBackLeft(double power, double inches, double idealHeading){
        //this is front right on other people's view of this bot -> FL and BR forwards
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        double multiplier;
        motorFL.setPower(power);
        motorBR.setPower(power);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) > -inches && opModeIsActive()){
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorFL.setPower(power * multiplier);
                motorBR.setPower(power);
                //so this means we need turn left...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorFL.setPower(power);
                motorBR.setPower(power * multiplier);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
        }
        stopMotors();
    }
    public void moveBackRight(double power, double inches, double idealHeading){
        //this is front left on other people's view of this bot -> FR and BL forwards
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        double multiplier;
        motorFR.setPower(power);
        motorBL.setPower(power);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) < inches && opModeIsActive()){
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorBL.setPower(power * multiplier);
                motorFR.setPower(power);
                //so this means we need turn left...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
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
   /* public double getInchesTraveled(DcMotor encoder, int encoderInitial){ //the version that takes encoderInitial in ticks
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
        //okay so encoder/1892 is the encoder's current reading in inches
        //and then we subtract encoderInitial which is... here... -14.3ish
    }*/
    public double newInchesTraveled(int startTicks, int endTicks){
        //okay this time we are going to do it in a REASONABLE WAY.
        //see, it's that easy!
        return (double)(endTicks - startTicks)/1892.0;
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
        RobotLog.aa("InitialArmPos", String.valueOf(armInitial));
        armDownPos = armInitial;
        armUpPos = armInitial-800;
        armSlightlyOffGroundPos = armInitial - 150;
        armBackdropDeliveryPos = armInitial - 480;
        RobotLog.aa("SlightlyOffArmPos", String.valueOf(armSlightlyOffGroundPos));
        armSpikeMarkPos = armInitial - 250; //used to be -100, then -200

        arm.setTargetPosition(armDownPos); //see, driver hub? I'M DOING WHAT YOU WANT
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        /*List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }*/
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        processor = new EverythingProcessor();
        ATProcessor = AprilTagProcessor.easyCreateWithDefaults();
        processor.setAlliance(1);
        portal = VisionPortal.easyCreateWithDefaults(webcam, processor, ATProcessor);
        portal.setProcessorEnabled(ATProcessor, false);
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
      processor.setMode(EverythingProcessor.ProcessorMode.PIXEL);
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
        int count = 0;
        while(opModeIsActive() && (/*Math.abs(pixelPos.x-320) > 10 ||*/ pixelPos.y < 340)){ //y threshold was 300
            //RobotLog.aa("DistanceFromCenter", String.valueOf(Math.abs(pixelPos.x - 320)));
            count++;
            double proportionalConstant = -.025; // used to be -.5, then -.3, then -.03, then -.01, then -.015, then -.03; Desmos said -0.00344828
            pixelPos = processor.getClosestPixelPos();
            //RobotLog.aa("PixelPos", String.valueOf(pixelPos));
            if(Math.abs(pixelPos.x - 320) < 10){ //we're close enough to centered to just go straight backwards
              //  RobotLog.aa("Motors", "all the same");
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
                //RobotLog.aa("multiplier", String.valueOf(multiplier));
                telemetry.addData("multiplier", Double.toString(multiplier));
                telemetry.update();
                //RobotLog.aa("Going", "left");
                //RobotLog.aa("Motors", "setting FL and BR to " + (-power * multiplier));
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
                //RobotLog.aa("multiplier", String.valueOf(multiplier));
                //RobotLog.aa("Going", "right");
                //RobotLog.aa("Motors", "decreasing FR and BL to " + (-power * multiplier));
                motorFL.setPower(-power * multiplierFL);
                motorBR.setPower(-power * multiplierBR);
                motorFR.setPower(-power * multiplierFR * multiplier);
                motorBL.setPower(-power * multiplierBL * multiplier);
            }
        }
        stopMotors();
        closeClaw();
    }
    public double[] getAprilTagDist(String result){
        //IDs: 1 is blue left, 2 is blue center, 3 is blue right
        //4 is red left, 5 is red center, 6 is red right
        List<AprilTagDetection> currentDetections = ATProcessor.getDetections();
        double xDist = 0.0;
        double yDist = 0.0;
        double[] dists = new double[2];
        for(AprilTagDetection d : currentDetections) {
            if (result.equals("Center")) {
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
            }
        }
        dists[0] = xDist;
        dists[1] = yDist;
        return dists;
    }

}
