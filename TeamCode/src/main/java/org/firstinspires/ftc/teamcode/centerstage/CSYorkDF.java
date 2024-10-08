package org.firstinspires.ftc.teamcode.centerstage;

import android.util.Size;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@Autonomous

public class CSYorkDF extends LinearOpMode {
    Blinker control_Hub;
    IMU imu;
    private WebcamName backCamera;
    private WebcamName frontCamera;
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx motorBL;
    DcMotorEx forwardOdo;
    DcMotorEx strafeOdo;
    SparkFunOTOS opticalOdo;
    DcMotorEx liftEncoder;
    DcMotorEx lift1;
    DcMotorEx lift2;

    Servo clawUp;
    Servo clawDown;
    Servo wrist;
    ServoImplEx arm1;
    ServoImplEx cameraBar;
    Servo endStop;
    VisionPortal portal;
    AnalogInput frontRightUltrasonic;
    AnalogInput backRightUltrasonic;
    AnalogInput frontLeftUltrasonic;
    AnalogInput backLeftUltrasonic;
    RevColorSensorV3 clawLeftSensor;
    RevColorSensorV3 clawRightSensor;
    RevColorSensorV3 backdropDetector;
    EverythingProcessor processor;
    AprilTagProcessor ATProcessor;
    int strafeInitialTicks;
    int forwardInitialTicks;
    double previousHeading;
    double processedHeading;
    double multiplierFR = 1.0;
    double multiplierBL = 1.0;
    double multiplierFL = 1.0;
    double multiplierBR = 1.0;
    double clawUpOpen = .465; //0.51; //was .525
    double clawUpInitClose = .675;
    double clawUpClose = 0.76;//.33; //0.355; //0.38 was good, then 0.79
    double clawUpLessOpen = .5;
    double clawUpSlightlyOpen = 0.62;
    double clawUpSuperOpen = .33;
    double clawDownOpen = 0.465;//.525; //0.53; //was .58 before servo broke
    double clawDownSemiClose = .62;
    double clawDownLessOpen = .5;
    double clawDownSlightlyOpen = 0.55;

    double clawDownInitClose = .61;
    double clawDownSuperOpen = .38;
    double clawDownClose = 0.635;//.385; //0.42; //was .47 before servo broke //0.435 was good but didn't stall enough, 0.69 stalled a lot
    double wristDownPos = 0.145; //was 0.135
    double wristAboveStackPos = .09; //.07
    double wristAlmostDown = 0.15;//for flipping the arm up
    double wristStraightUp = 0.45;
    double wristTuckedIn = 0.735;
    double wristScoringPos = 0.54;
    double arm1ScoringPos = 0.07;//was 0.1
    double arm1AboveStackPos = .72;
    double arm1AlmostUp = .2; //was 0.37, then 0.2025, then .345, then .175
    double arm1AlmostDown = 0.65; // was 0.6, .7 was too much, 0.65 was too low, then 0.6
    double arm1DownPos = 0.82; //.8
    double arm1InitDownPos = 0.805;
    double arm2ScoringPos = 0.08;
    double arm2AlmostUp = 0.175; //was .345, then .175
    double arm2AlmostDown = 0.61; //was 0.61, .71 was too much, 0.66 was too low
    double arm2DownPos = 0.7975;
    double arm145 = 0.785;
    double arm245 = 0.785;
    double armStack45Pos = .747; //was .935 then .93 then .937 then arm servo broke then .767
    double armStack34Pos = 0.777; //was .945 then .947 then arm servo broke
    double armStack23Pos = 0.785; //was .955, then .96, then .957, then .955, then arm servo broke
    double armStallAgainstStopPos = 0.83; // was 1.0 for old servo
    //the 4 previous values are OUTDATED! OLD! FROM BEFORE THE SERVO BROKE! FROM BEFORE WE USED TWO SERVOS! do not use.
    double endStopOutOfWayPos = .64; //was .64
    double endStop45Pos = .53; //was 0.545, then .53, then .525, then .52, then 0.515
    double endStop34Pos = .565; //was 0.565, then .57, then .565
    double endStop23Pos = .6; //was .58, then .605
    double newMotorsConstant = 2.8;
    double liftYellowPixelPos = .04 * newMotorsConstant; //0.035; //0.0262024407753051;
    double liftFirstWhitePixelPos = .04 * newMotorsConstant;
    double liftSecondWhitePixelPos = .08 * newMotorsConstant;
    double liftTolerance = 0.00375 * newMotorsConstant; //was 0.005
    double wristStack45Pos = 0.13; //was 0.12, then 0.115
    double wristStack34Pos = 0.13; //was 0.12. 0.12 is a bit off, and 0.125 is a bit off the other way, so 0.1225
    double wristStack23Pos = 0.135; // was 0.13
    //these ^^^ may need changing -- we'll see
    double camOutOfWay = 0.36; //pointing straight out
    double camUsePos = 0.6475;
    double camTuckedIn = 0.9575;
    double liftPos;
    double liftIdealPos;
    double liftInitial;
    boolean liftHappyPlace = true;
    double ticksPerRotation;
    Point pixelPos;
    double inchTolerance;
    double degreeTolerance;
    public void runOpMode(){
        initializeHardware();
        openClaw();
        //while(portal.getCameraState() != CameraState.STREAMING && opModeInInit()){

        //}
        //telemetry.addData("Status", "Actually ready");
        //telemetry.update();
        waitForStart();
        //activateBackCamera();
        //centerOnClosestStack(processor);
        double biggest = Double.MIN_VALUE;
        double smallest = Double.MAX_VALUE;
        ArrayList<Double> rightAverages = new ArrayList<>();
        ArrayList<Double> centerAverages = new ArrayList<>();
        ArrayList<Double> leftAverages = new ArrayList<>();
        double leftAvg = 0.0;
        double centerAvg = 0.0;
        double rightAvg = 0.0;
        while(opModeIsActive()){
            //telemetry.addData("AprilTagsLeft", Arrays.toString(getAprilTagDist("Left")));
            //telemetry.addData("AprilTagsCenter", Arrays.toString(getAprilTagDist("Center")));
            //telemetry.addData("AprilTagsRight", Arrays.toString(getAprilTagDist("Right")));
            telemetry.addData("Heading", newGetHeading());
            telemetry.addData("StrafeTicks", strafeOdo.getCurrentPosition());
            telemetry.addData("ForwardTicks", forwardOdo.getCurrentPosition());
            //telemetry.addData("LeftClaw", clawLeftSensor.getDistance(DistanceUnit.INCH));
            //telemetry.addData("RightClaw", clawRightSensor.getDistance(DistanceUnit.INCH));
            //30 inches is seeing the truss
            telemetry.update();
        }
        portal.close();
    }
    public void goStraight(double power, double inches, double idealHeading){
        double multiplier;
        //double processedInches = -1 * inches;
        int forwardBackStartTicks = forwardOdo.getCurrentPosition();
        telemetry.addData("StartTicks", forwardBackStartTicks);
        int forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("GoStraight", "goal heading is " + targetheading);
        while(newInchesTraveled(forwardBackStartTicks, forwardBackCurrentTicks) < inches && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            //RobotLog.aa("CurrentHeading", String.valueOf(heading));
            if(heading-targetheading<0){  //we need to turn left
                //RobotLog.aa("Going", "Left");
                multiplier = -.1*(heading-targetheading)+1;
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
                motorBR.setPower(power * multiplier);
            }else if(heading-targetheading>=0){
                //RobotLog.aa("Going", "Right");
                multiplier = .1*(heading-targetheading)+1;
                motorFR.setPower(power);
                motorBR.setPower(power);
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
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
        motorFR.setPower(-power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(-power);
        double targetheading = idealHeading;
        RobotLog.aa("GoBackward", "goal heading is " + targetheading);
        while(newInchesTraveled(forwardBackStartTicks, forwardBackCurrentTicks) > -inches && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            if(heading-targetheading < 0){ //turn to the left
                multiplier = -.1*(heading-targetheading)+1;
                motorFL.setPower(-power*multiplier);
                motorBL.setPower(-power*multiplier);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
            }else if(heading-targetheading >= 0){ //turn to the right
                multiplier = .1*(heading-targetheading)+1;
                motorFR.setPower(-power*multiplier);
                motorBR.setPower(-power*multiplier);
                motorFL.setPower(-power);
                motorBL.setPower(-power);
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
        //RobotLog.aa("Initial", String.valueOf(strafeStartTicks));
        //RobotLog.aa("InitialCurrent", String.valueOf(strafeCurrentTicks));
        //RobotLog.aa("Third", String.valueOf(strafeThirdTicks));
        //RobotLog.aa("InitialDifference", String.valueOf(strafeCurrentTicks - strafeStartTicks));
        //RobotLog.aa("InitialInches", String.valueOf(newInchesTraveled(strafeStartTicks, strafeCurrentTicks)));
        motorFR.setPower(-power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(-power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeRight", "goal heading is " + targetheading);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) > -inches && current-startthing < 1000*timelimit && opModeIsActive()){
            liftWithinLoop();
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
    public void strafeLeft(double power, double inches, double timelimit, double idealHeading){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int strafeThirdTicks = strafeOdo.getCurrentPosition();
       // RobotLog.aa("Initial", String.valueOf(strafeStartTicks));
        //RobotLog.aa("InitialCurrent", String.valueOf(strafeCurrentTicks));
        //RobotLog.aa("Third", String.valueOf(strafeThirdTicks));
        //RobotLog.aa("InitialDifference", String.valueOf(strafeCurrentTicks - strafeStartTicks));
        //RobotLog.aa("InitialInches", String.valueOf(newInchesTraveled(strafeStartTicks, strafeCurrentTicks)));
        motorFR.setPower(power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeLeft", "goal heading is " + targetheading);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) < inches && current - startthing < 1000*timelimit && opModeIsActive()){
            liftWithinLoop();
            //RobotLog.aa("Inches", String.valueOf(newInchesTraveled(strafeStartTicks, strafeCurrentTicks)));
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
        RobotLog.aa("Status", "Stopping");
        stopMotors();
    }
    public double moveForwardRight(double power, double inches, double idealHeading){ //returns the number of inches it moved forward
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int forwardStartTicks = forwardOdo.getCurrentPosition();
        double multiplier;
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorFR.setPower(-power/2);
        motorBL.setPower(-power/2);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) > -inches && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorFL.setPower(power * multiplier);
                motorBR.setPower(power);
                motorFR.setPower(-power/2);
                motorBL.setPower(-power/2);
                //so this means we need turn right...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorFL.setPower(power);
                motorBR.setPower(power * multiplier);
                motorFR.setPower(-power/2);
                motorBL.setPower(-power/2);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
        }
        stopMotors();
        int forwardEndTicks = forwardOdo.getCurrentPosition();
        return newInchesTraveled(forwardStartTicks, forwardEndTicks);
    }
    public double moveForwardLeft(double power, double inches, double idealHeading){ //returns number of inches moved forwards
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int forwardStartTicks = forwardOdo.getCurrentPosition();
        double multiplier;
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorFL.setPower(-power/2);
        motorBR.setPower(-power/2);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) < inches && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorBL.setPower(power * multiplier);
                motorFR.setPower(power);
                motorFL.setPower(-power/2);
                motorBR.setPower(-power * multiplier /2);
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
                motorFL.setPower(-power * multiplier /2);
                motorBR.setPower(-power/2);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
            //RobotLog.aa("Strafe", String.valueOf(strafeCurrentTicks));
        }
        stopMotors();
        int forwardEndTicks = forwardOdo.getCurrentPosition();
        return newInchesTraveled(forwardStartTicks, forwardEndTicks);
    }
    public double moveBackLeft(double power, double inches, double idealHeading){ //returns number of inches moved (will be negative)
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int forwardStartTicks = forwardOdo.getCurrentPosition();
        double multiplier;
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorFR.setPower(power/2);
        motorBL.setPower(power/2);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) < inches && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorFL.setPower(-power);
                motorBR.setPower(-power * multiplier);
                motorFR.setPower(power/2);
                motorBL.setPower(power/2);
                //so this means we need turn left...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorFL.setPower(-power * multiplier);
                motorBR.setPower(- power);
                motorFR.setPower(power/2);
                motorBL.setPower(power/2);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
        }
        stopMotors();
        int forwardEndTicks = forwardOdo.getCurrentPosition();
        return newInchesTraveled(forwardStartTicks, forwardEndTicks);
    }
    public double moveBackRight(double power, double inches, double idealHeading){
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int forwardStartTicks = forwardOdo.getCurrentPosition();
        double multiplier;
        motorFR.setPower(-power);
        motorBL.setPower(-power);
        motorFL.setPower(power/2);
        motorBR.setPower(power/2);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) > -inches && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorBL.setPower(-power);
                motorFR.setPower(-power * multiplier);
                motorFL.setPower(power/2);
                motorBR.setPower(power/2);
                //so this means we need turn left...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorBL.setPower(-power * multiplier);
                motorFR.setPower(-power);
                motorFL.setPower(power/2);
                motorBR.setPower(power/2);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
        }
        stopMotors();
        int forwardEndTicks = forwardOdo.getCurrentPosition();
        return newInchesTraveled(forwardStartTicks, forwardEndTicks);
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
    public void goStraightForTime(double power, double seconds, double idealHeading){
        double multiplier;
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("GoStraight", "goal heading is " + targetheading);
        long startTime = System.nanoTime();
        long currentTime = System.nanoTime();
        while((double)(currentTime-startTime)/Math.pow(10, 9) < seconds && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            //RobotLog.aa("CurrentHeading", String.valueOf(heading));
            if(heading-targetheading<0){  //we need to turn left
                //RobotLog.aa("Going", "Left");
                multiplier = -.1*(heading-targetheading)+1;
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
                motorBR.setPower(power * multiplier);
            }else if(heading-targetheading>=0){
                //RobotLog.aa("Going", "Right");
                multiplier = .1*(heading-targetheading)+1;
                motorFR.setPower(power);
                motorBR.setPower(power);
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
            }
            currentTime = System.nanoTime();
        }
        stopMotors();
    }
    public void goStraightWithLimit(double power, double inches, double timeLimit, double idealHeading){
        double multiplier;
        //double processedInches = -1 * inches;
        int forwardBackStartTicks = forwardOdo.getCurrentPosition();
        telemetry.addData("StartTicks", forwardBackStartTicks);
        int forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("GoStraight", "goal heading is " + targetheading);
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        while(newInchesTraveled(forwardBackStartTicks, forwardBackCurrentTicks) < inches && (currentTime-startTime) < timeLimit*1000 && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            //RobotLog.aa("CurrentHeading", String.valueOf(heading));
            if(heading-targetheading<0){  //we need to turn left
                //RobotLog.aa("Going", "Left");
                multiplier = -.1*(heading-targetheading)+1;
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
                motorBR.setPower(power * multiplier);
            }else if(heading-targetheading>=0){
                //RobotLog.aa("Going", "Right");
                multiplier = .1*(heading-targetheading)+1;
                motorFR.setPower(power);
                motorBR.setPower(power);
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
            }
            currentTime = System.currentTimeMillis();
            forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        }
        stopMotors();
    }

    public void strafeRightUntilPixel(double power, double idealHeading){
        double multiplier;
        motorFR.setPower(-power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(-power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeRight", "goal heading is " + targetheading);
        boolean isDone = (clawLeftSensor.getDistance(DistanceUnit.INCH) < 2.5 || clawRightSensor.getDistance(DistanceUnit.INCH) < 2.7);
        while(!isDone && opModeIsActive()){
            liftWithinLoop();
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
            double clawLeft = clawLeftSensor.getDistance(DistanceUnit.INCH);
            double clawRight = clawRightSensor.getDistance(DistanceUnit.INCH);
            RobotLog.aa("ClawLeft", String.valueOf(clawLeft));
            RobotLog.aa("ClawRight", String.valueOf(clawRight));
            isDone = (clawLeft < 2.5 || clawRight < 2.7);
        }
        stopMotors();
    }
    public void strafeLeftUntilPixel(double power, double idealHeading){
        double multiplier;
        motorFR.setPower(power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeLeft", "goal heading is " + targetheading);
        boolean isDone = (clawLeftSensor.getDistance(DistanceUnit.INCH) < 2.5 || clawRightSensor.getDistance(DistanceUnit.INCH) < 2.7);
        while(!isDone && opModeIsActive()){
            liftWithinLoop();
            //RobotLog.aa("Inches", String.valueOf(newInchesTraveled(strafeStartTicks, strafeCurrentTicks)));
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
            double clawLeft = clawLeftSensor.getDistance(DistanceUnit.INCH);
            double clawRight = clawRightSensor.getDistance(DistanceUnit.INCH);
            RobotLog.aa("ClawLeft", String.valueOf(clawLeft));
            RobotLog.aa("ClawRight", String.valueOf(clawRight));
            isDone = (clawLeft < 2.5 || clawRight < 2.7);
        }
        RobotLog.aa("Status", "Stopping");
        stopMotors();
    }
    public double moveBackRightMoreStraight(double power, double inches, double idealHeading){
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int forwardStartTicks = forwardOdo.getCurrentPosition();
        double multiplier;
        motorFR.setPower(-power);
        motorBL.setPower(-power);
        motorFL.setPower(power/4);
        motorBR.setPower(power/4);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) > -inches && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorBL.setPower(-power);
                motorFR.setPower(-power * multiplier);
                motorFL.setPower(power/4);
                motorBR.setPower(power/4);
                //so this means we need turn left...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorBL.setPower(-power * multiplier);
                motorFR.setPower(-power);
                motorFL.setPower(power/4);
                motorBR.setPower(power/4);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
        }
        stopMotors();
        int forwardEndTicks = forwardOdo.getCurrentPosition();
        return newInchesTraveled(forwardStartTicks, forwardEndTicks);
    }
    public double moveBackLeftMoreStraight(double power, double inches, double idealHeading){ //returns number of inches moved (will be negative)
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks = strafeOdo.getCurrentPosition();
        int forwardStartTicks = forwardOdo.getCurrentPosition();
        double multiplier;
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorFR.setPower(power/4);
        motorBL.setPower(power/4);
        while(newInchesTraveled(strafeStartTicks, strafeCurrentTicks) < inches && opModeIsActive()){
            liftWithinLoop();
            double heading = newGetHeading();
            if(heading-idealHeading >= 0){
                multiplier = .1*(heading-idealHeading)+1;
                motorFL.setPower(-power);
                motorBR.setPower(-power * multiplier);
                motorFR.setPower(power/4);
                motorBL.setPower(power/4);
                //so this means we need turn left...
                //make FL more
            }else{
                multiplier = -.1*(heading-idealHeading)+1;
                motorFL.setPower(-power * multiplier);
                motorBR.setPower(-power);
                motorFR.setPower(power/4);
                motorBL.setPower(power/4);
            }
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
        }
        stopMotors();
        int forwardEndTicks = forwardOdo.getCurrentPosition();
        return newInchesTraveled(forwardStartTicks, forwardEndTicks);
    }
    public boolean placeAndHeading(double startX, double startY, double x, double y, double idealHeading, double powerMult, double inTol, double degTol, boolean stop, boolean isQuadrant) {
        processedHeading = newGetHeading();
        double rxConst = 13; //was 15, then 13, then 12
        double moveConst = 2; //maybe needs editing; was 1, then 1.5
        double currentX = opticalOdo.getPosition().x;
        double currentY = opticalOdo.getPosition().y;
        double forceVectorCorrection = 0.42; //was 0.445
        telemetry.addData("currentX, currentY", currentX + ", " + currentY);
        double xDifference = currentX - x;
        double yDifference = currentY - y;
        telemetry.addData("xError", xDifference);
        telemetry.addData("yError", yDifference);
        RobotLog.aa("xDiff_yDiff", xDifference + ", " + yDifference);
        double l = Math.sqrt(xDifference*xDifference + yDifference*yDifference); // diagonal error
        telemetry.addData("Diagonal error", l);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //x = fake joystick left left/right (strafe)
        double joyX = -1 * xDifference / l;
        //telemetry.addData("joyX", joyX);
        //y = fake joystick left up/down (move)
        double joyY = -1 * yDifference / l;
        //telemetry.addData("joyY", joyY);
        if (l < inTol) {
            joyX = 0;
            joyY = 0;
        }
        if (l < 5) {
            moveConst = 2 * l/5; //was l/5
        }
        joyX *= moveConst;
        joyY *= moveConst; //used to be inside the if; testing what happens if it's outside
        //rx = fake joystick right left/right aka turning
        double rx = 0;
        double rotError = Math.abs(processedHeading % 360 - idealHeading);
        if(rotError < 5){ //attempting to account for the rotational jerk
            rxConst = 13 * rotError/5; //should be 12 at 5 degrees, so... rotError*12/5
        }
        telemetry.addData("HeadingError", rotError);
        if (stop && l < inTol && rotError < degTol) { //stop condition for when this is the endpoint of a move
            //if we actually don't need to do anything
            stopMotors();
            return true;
        }
        if(!stop){ //stop condition for when this is not the endpoint of a move
            boolean good = false;
            if(isQuadrant){ //we want to be past the point in both directions of the move
                //find direction of comparison for x and y
                boolean isGoingForward = y >= startY;
                boolean isGoingRight = x >= startX;
                if(isGoingForward){
                    if(isGoingRight){
                        good = currentY > y && currentX > x;
                    }else{
                        good = currentY > y && currentX < x;
                    }
                }else{
                    if(isGoingRight){
                        good = currentY < y && currentX > x;
                    }else{
                        good = currentY < y && currentX < x;
                    }
                }
            }else{ //we want to be past the point in the larger direction of move?
                boolean isX = Math.abs(startX-x) > Math.abs(startY-y);
                //now we know the larger direction
                boolean isPositive;
                if(isX) isPositive = x>startX;
                else isPositive = y>startY;
                if(isX){
                    if(isPositive){
                        good = currentX > x;
                    }else{
                        good = currentX < x;
                    }
                }else{
                    if(isPositive){
                        good = currentY > y;
                    }else{
                        good = currentY < y;
                    }
                }
            }
            if(l < inTol) good = true;
            if(good) return true;
        }
        double sign = rotError/(processedHeading % 360 - idealHeading);
        if (rotError > 45) {rxConst = 70;} //used to be rxConst = 90
        if (rotError < degTol) {
            rx = 0;
        } else if (sign == -1) {
            //turning left
            rx = -1 * (rxConst/180) * rotError; //used to be rxConst * 180
        } else if (sign == 1) {
            //turning right
            rx = (rxConst/180) * rotError;
        }
        //cap rx if too large
        if (rx < -1) {rx = -1;}
        if (rx > 1) {rx = 1;}
        //scale up rx if too small
        if(Math.abs(rx) < .2 && Math.abs(rotError) > degTol){
            if(rx > 0) rx = .2;
            else if(rx < 0) rx = -.2;
        }
        //joyY *= forceVectorCorrection; //correcting for the fact that gobilda mecanum force vectors are 66 degrees not 45
        //force vector correction does NOT work on non-0 headings
        //code for capping joyX, joyY to real, possible joystick values that cause movement
        //if values are too small and there's still error
        double joyTol = .3;
        if(Math.abs(l) > inTol && Math.abs(joyX) < joyTol && Math.abs(joyY) < joyTol){ //these values for joystick-too-small are *completely* arbitrary
            int quadrant = 1;
            if(joyX < 0){
                if(joyY > 0){
                    quadrant = 2;
                }else{
                    quadrant = 3;
                }
            }else if(joyY < 0){
                quadrant = 4;
            }
            //sqrt(newX^2 + newY^2) = 1
            //angle between (newX, newY) and center is arctan(joyY/joyX)
            double angle = Math.atan(joyY/joyX);
            double innerCircleRadius = .3; //was .5, then .4
            if(angle < 0 && quadrant == 2){
                angle += Math.PI;
                joyX = Math.cos(angle) * innerCircleRadius;
                joyY = Math.sin(angle) * innerCircleRadius;
            }else if(angle < 0 && quadrant == 4){
                joyX = Math.cos(angle) * innerCircleRadius;
                joyY = Math.sin(angle) * innerCircleRadius;
            }else if(angle > 0 && quadrant == 1){
                //this is the easiest one
                joyX = Math.cos(angle) * innerCircleRadius;
                joyY = Math.sin(angle) * innerCircleRadius;
            }else if(angle > 0 && quadrant == 3){
                angle += Math.PI;
                joyX = Math.cos(angle * innerCircleRadius);
                joyY = Math.sin(angle * innerCircleRadius);
            }
        }
        //if values are too large
        if(joyX == 0 || joyY == 0){
            if(joyX == 0 && joyY != 0){
                if(joyY > 1) joyY = 1;
                if(joyY < -1) joyY = -1;
            }else if (joyX != 0 && joyY == 0){
                if(joyX < -1) joyX = -1;
                if(joyX > 1) joyX = 1;
            }
        }else if(Math.sqrt(Math.pow(joyX, 2) + Math.pow(joyY, 2)) > 1){ //the madness begins
            int quadrant = 1;
            if(joyX < 0){
                if(joyY > 0){
                    quadrant = 2;
                }else{
                    quadrant = 3;
                }
            }else if(joyY < 0){
                quadrant = 4;
            }
            //sqrt(newX^2 + newY^2) = 1
            //angle between (newX, newY) and center is arctan(joyY/joyX)
            double angle = Math.atan(joyY/joyX);
            if(angle < 0 && quadrant == 2){
                angle += Math.PI;
                joyX = Math.cos(angle);
                joyY = Math.sin(angle);
            }else if(angle < 0 && quadrant == 4){
                joyX = Math.cos(angle);
                joyY = Math.sin(angle);
            }else if(angle > 0 && quadrant == 1){
                //this is the easiest one
                joyX = Math.cos(angle);
                joyY = Math.sin(angle);
            }else if(angle > 0 && quadrant == 3){
                angle += Math.PI;
                joyX = Math.cos(angle);
                joyY = Math.sin(angle);
            }
        }
        telemetry.addData("joyX", joyX);
        telemetry.addData("joyY", joyY);
        telemetry.addData("rx", rx);
        double rotX = joyX * Math.cos(-botHeading) - joyY * Math.sin(-botHeading);
        double rotY = joyX * Math.sin(-botHeading) + joyY * Math.cos(-botHeading);
        double innerCircleRadius = Math.sqrt(Math.pow(rotX, 2) + Math.pow(rotY, 2));
        rotY *= forceVectorCorrection;
        //Below: Scale current thing to the magnitude of the pre-correction thing
        int quadrant = 1;
        if(rotX < 0){
            if(rotY > 0){
                quadrant = 2;
            }else{
                quadrant = 3;
            }
        }else if(rotY < 0){
            quadrant = 4;
        }
        double angle = Math.atan(rotY/rotX);
        if(angle < 0 && quadrant == 2){
            angle += Math.PI;
            rotX = Math.cos(angle) * innerCircleRadius;
            rotY = Math.sin(angle) * innerCircleRadius;
        }else if(angle < 0 && quadrant == 4){
            rotX = Math.cos(angle) * innerCircleRadius;
            rotY = Math.sin(angle) * innerCircleRadius;
        }else if(angle > 0 && quadrant == 1){
            //this is the easiest one
            rotX = Math.cos(angle) * innerCircleRadius;
            rotY = Math.sin(angle) * innerCircleRadius;
        }else if(angle > 0 && quadrant == 3){
            angle += Math.PI;
            rotX = Math.cos(angle * innerCircleRadius);
            rotY = Math.sin(angle * innerCircleRadius);
        }
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        motorFL.setPower(powerMult * frontLeftPower);
        motorBL.setPower(powerMult * backLeftPower);
        motorFR.setPower(powerMult * frontRightPower);
        motorBR.setPower(powerMult * backRightPower);
        telemetry.addData("FRPower", powerMult*frontRightPower);
        telemetry.addData("FLPower", powerMult*frontLeftPower);
        telemetry.addData("BRPower", powerMult*backRightPower);
        telemetry.addData("BLPower", powerMult*backLeftPower);
        RobotLog.aa("FL_BL_FR_BR", (powerMult*frontLeftPower)+ ", " + (powerMult*backLeftPower) + ", " + (powerMult*frontRightPower) + ", " + (powerMult+backRightPower));
        telemetry.update();
        return false;
    }
    public void moveTo(double power, Position pos, boolean stop){
       moveTo(power, pos, stop, true, 5.0);
    }
    public void moveTo(double power, Position pos, boolean stop, double timeLimit) {
        moveTo(power, pos, stop, true, timeLimit);
    }
    public void moveTo(double power, Position pos, boolean stop, boolean isQuadrant){
        moveTo(power, pos, stop, isQuadrant, 5.0);
    }
    public void moveTo(double power, Position pos, boolean stop, boolean isQuadrant, double timeLimit){
        //should there be a default timeout? 5 seconds or something like that.
        //timeLimit in seconds
        double startX = opticalOdo.getPosition().x;
        double startY = opticalOdo.getPosition().y;
        long prevTime = System.currentTimeMillis();
        long startTime = System.currentTimeMillis();
        while(opModeIsActive()){
            if(placeAndHeading(startX, startY, pos.getX(), pos.getY(), pos.getHeading(), power, inchTolerance, degreeTolerance, stop, isQuadrant)) return;
            //inTol used to be .5; testing to see if this stops the overcorrection problem
            long nowTime = System.currentTimeMillis();
            if(nowTime - startTime > timeLimit*1000){
                if(stop) stopMotors();
                break;
            }
            telemetry.addData("LoopTime", nowTime-prevTime);
            RobotLog.aa("LoopTime", String.valueOf(nowTime-prevTime));
            prevTime = nowTime;
        }
    }
    public void gyroTurn(double power, double degrees){ //right is negative
        if(opModeIsActive()){
            double gyroinitial = newGetHeading();
            if(degrees>0){ //turn left
                while(newGetHeading() - gyroinitial < degrees && opModeIsActive()){
                    liftWithinLoop();
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
                    liftWithinLoop();
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
    public void closeUpperClaw(){
        clawUp.setPosition(clawUpClose);
    }
    public void openUpperClaw(){
        clawUp.setPosition(clawUpOpen);
    }
    public void closeLowerClaw(){
        clawDown.setPosition(clawDownClose);
    }
    public void openLowerClaw(){
        clawDown.setPosition(clawDownOpen);
    }
    public void closeClaw(){
        closeUpperClaw();
        closeLowerClaw();
    }
    public void openClaw(){
        openUpperClaw();
        openLowerClaw();
    }
    public void superOpenClaw(){
        clawUp.setPosition(clawUpSuperOpen);
        clawDown.setPosition(clawDownSuperOpen);
    }
    public void activateFrontCamera(){
        if(portal.getCameraState() == CameraState.STREAMING){
            portal.setActiveCamera(frontCamera);
            if(!portal.getProcessorEnabled(processor)) portal.setProcessorEnabled(processor, true);
            if(portal.getProcessorEnabled(ATProcessor)) portal.setProcessorEnabled(ATProcessor, false);
        }
    }
    public void activateBackCamera(){
        if(portal.getCameraState() == CameraState.STREAMING){
            portal.setActiveCamera(backCamera);
            if(!portal.getProcessorEnabled(ATProcessor)) portal.setProcessorEnabled(ATProcessor, true);
            if(portal.getProcessorEnabled(processor)) portal.setProcessorEnabled(processor, false);
        }
    }
    public void stopMotors(){
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }
    public static double newInchesTraveled(int startTicks, int endTicks){
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
    public void wait(int time){
        long start = System.nanoTime();
        long now = System.nanoTime();
        while(now - start < (time*Math.pow(10, 6)) && opModeIsActive()){
            liftWithinLoop();
            now = System.nanoTime();
        }
    }
    public void setInchTolerance(double i){inchTolerance = i;}
    public void setDegreeTolerance(double d){degreeTolerance = d;}
    public void initializeHardware(){
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardEncoder");
        forwardOdo = hardwareMap.get(DcMotorEx.class, "motorFRandForwardEncoder");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        strafeOdo = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        opticalOdo = hardwareMap.get(SparkFunOTOS.class, "optical");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandForwardOdo");
        lift1 = hardwareMap.get(DcMotorEx.class, "slideMotorL");
        lift2 = hardwareMap.get(DcMotorEx.class, "slideMotorR");
        ticksPerRotation = liftEncoder.getMotorType().getTicksPerRev();
        liftInitial = liftEncoder.getCurrentPosition()/ticksPerRotation;
        liftIdealPos = liftInitial;
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        strafeInitialTicks = strafeOdo.getCurrentPosition();
        forwardInitialTicks = forwardOdo.getCurrentPosition();
        //configure the optical odo
        opticalOdo.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        opticalOdo.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -0.66, 180);
        opticalOdo.setOffset(offset);
        opticalOdo.setLinearScalar(0.986333);
        opticalOdo.setAngularScalar(0.995088);
        opticalOdo.calibrateImu();
        opticalOdo.resetTracking();
        clawUp = hardwareMap.get(Servo.class, "claw0");
        clawDown = hardwareMap.get(Servo.class, "claw2");
        arm1 = hardwareMap.get(ServoImplEx.class, "arm3"); //this is the one that DOES have an encoder
        wrist = hardwareMap.get(Servo.class, "wrist");
        cameraBar = hardwareMap.get(ServoImplEx.class, "frontCamera");
        endStop = hardwareMap.get(Servo.class, "endStop");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        /*List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }*/
        backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        frontRightUltrasonic = hardwareMap.get(AnalogInput.class, "frontRightUltra");
        backRightUltrasonic = hardwareMap.get(AnalogInput.class, "backRightUltra");
        frontLeftUltrasonic = hardwareMap.get(AnalogInput.class, "frontLeftUltra");
        backLeftUltrasonic = hardwareMap.get(AnalogInput.class, "backLeftUltra");
        clawLeftSensor = hardwareMap.get(RevColorSensorV3.class, "clawLeft");
        clawRightSensor = hardwareMap.get(RevColorSensorV3.class, "clawRight");
        backdropDetector = hardwareMap.get(RevColorSensorV3.class, "backdropDetector");
        processor = new EverythingProcessor();
        ATProcessor = AprilTagProcessor.easyCreateWithDefaults();
        CameraName webcam = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(frontCamera, backCamera);
        degreeTolerance = 1;
        inchTolerance = 1;
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 360))
                .addProcessor(processor)
                .addProcessor(ATProcessor)
                .enableLiveView(false)
                .build();
        if(portal.getProcessorEnabled(ATProcessor)) portal.setProcessorEnabled(ATProcessor, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
    public void centerOnClosestStack(EverythingProcessor processor){
        if(!processor.getIsSeeingPixel()){
            goBackward(.4, 1);
            sleep(100);
            if(!processor.getIsSeeingPixel()){
                RobotLog.aa("Vision", "failed");
                endStop.setPosition(endStopOutOfWayPos);
                sleep(1000);
                armDown();
                sleep(30000);
            }
        }
        boolean happy = closestStackInnerFunction(processor);
        boolean tooFar = false;
        int strafeStart = strafeOdo.getCurrentPosition();
        int strafeCurrent = strafeOdo.getCurrentPosition();
        int forwardStart = forwardOdo.getCurrentPosition();
        int forwardCurrent = forwardOdo.getCurrentPosition();
        double strafeInches = newInchesTraveled(strafeStart, strafeCurrent);
        while(!happy && !tooFar && opModeIsActive()){
            happy = closestStackInnerFunction(processor);
            strafeCurrent = strafeOdo.getCurrentPosition();
            forwardCurrent = forwardOdo.getCurrentPosition();
            strafeInches = newInchesTraveled(strafeStart, strafeCurrent);
            tooFar = Math.abs(strafeInches) > 9;
        }
        if(tooFar){
            RobotLog.aa("Status", "Gone too far");
            double forwardInches = Math.abs(newInchesTraveled(forwardStart, forwardCurrent));
            goBackward(.4, forwardInches);
            //right is negative
            if(strafeInches < 0){
                RobotLog.aa("Status", "Undoing moves by going left");
                strafeLeft(.4, (-strafeInches), 5);
            }else{
                RobotLog.aa("Status", "Undoing moves by going right");
                strafeRight(.4, strafeInches, 5);
            }
            endStop.setPosition(endStopOutOfWayPos);
            sleep(1000);
            armDown();
            sleep(30000);
        }else {
            stopMotors();
            wait(500);
            //goBackward(.4, .05);
            closeClaw();
        }
    }
    public void armDown(){
        arm1.setPosition(arm1DownPos);
    }
    public void armAboveStack(){ arm1.setPosition(arm1AboveStackPos); }
    public void armAlmostDown(){
        arm1.setPosition(arm1AlmostDown);
    }
    public void armAlmostUp(){
        arm1.setPosition(arm1AlmostUp);
    }
    public void armUp(){
        arm1.setPosition(arm1ScoringPos);
    }
    public void stallArm(){
        arm1.setPosition(arm145);
    }
    public void liftWithinLoop(){
        liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);
        double liftError = liftIdealPos - liftPos;
        double liftStallPower = 0.4 * liftPos; //m was 0.41
        if (liftStallPower > 0.2) {
            liftStallPower = 0.2;
        }
        double Kp = 16;
        if (Math.abs(liftError) > liftTolerance) {
            if (liftError < 0) {
                Kp = 12; //was 11
            }
            if (liftError < 0 && liftIdealPos == 0) {
                Kp = 14;
            }
            lift2.setPower(liftError*Kp + liftStallPower);
            lift1.setPower(-liftError*Kp + liftStallPower);
        } else {
            lift2.setPower(liftStallPower);
            lift1.setPower(-liftStallPower);
        }
    }
    public double[] getAprilTagDist(String result){
        //IDs: 1 is blue left, 2 is blue center, 3 is blue right
        //4 is red left, 5 is red center, 6 is red right
        List<AprilTagDetection> currentDetections = ATProcessor.getDetections();
        if(currentDetections.size() == 0){
            sleep(100);
            currentDetections = ATProcessor.getDetections();
            if(currentDetections.size() == 0){
                sleep(100);
                currentDetections = ATProcessor.getDetections();
                if(currentDetections.size() == 0){
                    return new double[2];
                }
            }
        }
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
            RobotLog.aa("ValsToAverage", valsToAverage.toString());
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
            RobotLog.aa("ValsToAverage", valsToAverage.toString());
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
            RobotLog.aa("ValsToAverage", valsToAverage.toString());
            for(double d : valsToAverage){
                xAvg += d;
            }
            xAvg /= valsToAverage.size();
        }
        RobotLog.aa("xAvg", Double.toString(xAvg));
        frontDistAvg /= currentDetections.size();
        dists[0] = xAvg;
        dists[1] = frontDistAvg;
        return dists;
    }

    public String getPropResult(double leftAv, double rightAv, String processorResult){
        String cameraResult = processorResult;
        return cameraResult;
    }
    public double ultraDistance(AnalogInput ultra){
        double volt = ultra.getVoltage();
        return (205.849 * volt - 28.0321)/2.54;
    }
    public double frontRightUltraDistance(){ //should return in inches if I'm doing this right
        return ultraDistance(frontRightUltrasonic);
    }
    public double backRightUltraDistance(){
        return ultraDistance(backRightUltrasonic);
    }
    public double frontLeftUltraDistance(){ return ultraDistance(frontLeftUltrasonic); }
    public double backLeftUltraDistance(){ return ultraDistance(backLeftUltrasonic); }
}
