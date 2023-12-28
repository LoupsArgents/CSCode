package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.List;

@Autonomous

public class CSYorkDF extends LinearOpMode {
    Blinker control_Hub;
    IMU imu;
    WebcamName webcam;
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx motorBL;
    DcMotorEx forwardOdo;
    DcMotorEx strafeOdo;
    DcMotorEx liftEncoder;
    VisionPortal portal;
    Rev2mDistanceSensor leftDistance;
    Rev2mDistanceSensor rightDistance;
    Rev2mDistanceSensor centerDistance;

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
    Point pixelPos;
    public void runOpMode(){
        initializeHardware();
        openClaw();
        //portal.stopStreaming();
        processor.setMode(1);
        waitForStart();
        centerOnClosestStack(processor);
        while(opModeIsActive()){

        }
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
    public void closeClaw(){} //make once we're all wired and configured
    public void openClaw(){} //make once we're all wired and configured
    public void stopMotors(){
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }
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
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        strafeInitialTicks = strafeOdo.getCurrentPosition();
        forwardInitialTicks = forwardOdo.getCurrentPosition();
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
        leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance");
        centerDistance = hardwareMap.get(Rev2mDistanceSensor.class, "centerDistance");
        processor = new EverythingProcessor();
        ATProcessor = AprilTagProcessor.easyCreateWithDefaults();
        processor.setAlliance(1);
        portal = VisionPortal.easyCreateWithDefaults(webcam, processor, ATProcessor);
        portal.setProcessorEnabled(ATProcessor, false);
        portal.resumeStreaming();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    public boolean closestStackInnerFunction(EverythingProcessor processor){
        if(pixelPos.y < 340){ //y threshold was 300
            //RobotLog.aa("DistanceFromCenter", String.valueOf(Math.abs(pixelPos.x - 320)));
            double power = .35;
            double multiplier = 1;
            double proportionalConstant = -.025; // used to be -.5, then -.3, then -.03, then -.01, then -.015, then -.03; Desmos said -0.00344828
            pixelPos = processor.getClosestPixelPos();
            //RobotLog.aa("PixelPos", String.valueOf(pixelPos));
            if(Math.abs(pixelPos.x - 320) < 10){ //we're close enough to centered to just go straight backwards
                //  RobotLog.aa("Motors", "all the same");
                motorBL.setPower(-power * multiplierBL);
                motorBR.setPower(-power * multiplierBR);
                motorFL.setPower(-power * multiplierFL);
                motorFR.setPower(-power * multiplierFR);
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
            return false;
        }else{
            return true;
        }
    }
    public void centerOnClosestStack(EverythingProcessor processor){
        boolean happy = closestStackInnerFunction(processor);
        while(!happy && opModeIsActive()){
            happy = closestStackInnerFunction(processor);
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
    public DistanceSensorResult getDistances(){
        double leftResult = readDistanceSensor(leftDistance);
        double centerResult = readDistanceSensor(centerDistance);
        double rightResult = readDistanceSensor(rightDistance);
        return new DistanceSensorResult(leftResult, centerResult, rightResult);
    }
    public double readDistanceSensor(Rev2mDistanceSensor sensor){
        double val = sensor.getDistance(DistanceUnit.INCH);
        if(val == 0.0 || val > 330){
            val = sensor.getDistance(DistanceUnit.INCH);
        }
        return val;
    }
    class DistanceSensorResult {
        private double leftVal;
        private double centerVal;
        private double rightVal;
        private int errorCode;
        public DistanceSensorResult(double leftV, double centerV, double rightV){
            this.leftVal = leftV;
            this.centerVal = centerV;
            this.rightVal = rightV;
            //errorCode is the number of sensors that are reading nonsense
            int err = 0;
            if(leftV < 5 || leftV > 330) err++;
            if(centerV < 5 || centerV > 330) err++;
            if(rightV < 5 || rightV > 330) err++;
            this.errorCode = err;
        }
        public double getLeftVal(){ return this.leftVal; }
        public double getCenterVal(){ return this.centerVal; }
        public double getRightVal(){ return this.rightVal; }
        public int getErrorCode(){ return this.errorCode; }

    }
}
