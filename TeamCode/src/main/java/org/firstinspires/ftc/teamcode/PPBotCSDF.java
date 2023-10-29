package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
@Autonomous
public class PPBotCSDF extends TeamPropRecognition {
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
    TeamPropProcessor processor;
    int armDownPos = 0; //was -100
    int armUpPos = -600; //was -1390, then -700
    int armSpikeMarkPos = -100;
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
    int strafeInitialTicks;
    int forwardInitialTicks;
    double previousHeading;
    double processedHeading;
    public void runOpMode(){
        initializeHardware();
        closeClaw();
        waitForStart();
        String propPosition = processor.getResult();
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

        }
    }
    public void goStraight(double power, double inches, double idealHeading){
        double multiplier;
        //double processedInches = -1 * inches;
        telemetry.addData("RawPos", forwardOdo.getCurrentPosition());
        double forwardBackStart = getInchesTraveled(forwardOdo, 0); //second used to be 0 but IDK why
        telemetry.addData("Start", forwardBackStart);
        //The variable above gives the position, in inches, that the encoder was in at the start of the function.
        double forwardBackPos = getInchesTraveled(forwardOdo, forwardBackStart);
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
            forwardBackPos = getInchesTraveled(forwardOdo, forwardBackStart);
        }
        stopMotors();
        telemetry.addData("Inches", getInchesTraveled(forwardOdo, forwardInitialTicks));
        telemetry.update();
    }
    public void goBackward(double power, double inches, double idealHeading){
        double multiplier;
        double forwardBackStart = getInchesTraveled(forwardOdo, 0); //second was 0 but IDK why
        telemetry.addData("Start", forwardBackStart);
        //The variable above gives the position, in inches, that the encoder was in at the start of the function.
        double forwardBackPos = getInchesTraveled(forwardOdo, forwardBackStart);
        telemetry.addData("FirstPos", forwardBackPos);
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
            forwardBackPos = getInchesTraveled(forwardOdo, forwardBackStart);
        }
        stopMotors();
    }
    //going to the right is positive ticks, so going to the left will be negative
    public void strafeRight(double power, double inches, double timelimit, double idealHeading){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        double strafeStart = getInchesTraveled(strafeOdo, 0); //second was 0 but IDK why
        double strafePos = getInchesTraveled(strafeOdo, strafeStart);
        motorFR.setPower(power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeRight", "goal heading is " + targetheading);
        while(strafePos < inches && current-startthing < 1000*timelimit && opModeIsActive()){
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
            strafePos = getInchesTraveled(strafeOdo, strafeStart);
            current = System.currentTimeMillis();
        }
        stopMotors();
    }
    public void strafeLeft(double power, double inches, double timelimit, double idealHeading){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        double strafeStart = getInchesTraveled(strafeOdo, 0); //second was 0 but IDK why
        double strafePos = getInchesTraveled(strafeOdo, strafeStart);
        motorFR.setPower(-power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(-power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeLeft", "goal heading is " + Double.toString(targetheading));
        while(strafePos > -1 * inches && current - startthing < 1000*timelimit && opModeIsActive()){
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
            strafePos = getInchesTraveled(strafeOdo, strafeStart);
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
    public double getInchesTraveled(DcMotor encoder, double encoderInitial){ //the version that takes encoderInitial in inches
        return ((double)(encoder.getCurrentPosition()) / 1892.0) - encoderInitial;
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
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        processor = new TeamPropProcessor();
        processor.setAlliance(1);
        portal = VisionPortal.easyCreateWithDefaults(webcam, processor);
        portal.resumeStreaming();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
