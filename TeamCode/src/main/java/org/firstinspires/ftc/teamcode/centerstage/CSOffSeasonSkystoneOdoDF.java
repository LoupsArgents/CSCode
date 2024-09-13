package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

@Disabled

public class CSOffSeasonSkystoneOdoDF extends LinearOpMode {
    Blinker control_Hub;
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor strafeOdo;
    int strafeInitialTicks;
    DcMotor forwardOdo;
    int forwardInitialTicks;
    double previousHeading;
    double processedHeading;
    //strafe pod is in port 0, forward/back pod is in port 3
    IMU imu;
    public void runOpMode(){
        initializeHardware();
        waitForStart();
        goStraight(.2, 7, 0.0);
        goBackward(.2, 7, 0.0);
        strafeRight(.3, 5, 5, 0.0);
        strafeLeft(.3, 5, 5, 0.0);
        while(opModeIsActive()){
            telemetry.addData("Ticks", strafeOdo.getCurrentPosition() - strafeInitialTicks);
            telemetry.addData("Inches", getInchesTraveled(strafeOdo, strafeInitialTicks));
            telemetry.update();
        }
    }
    public void goStraight(double power, double inches, double idealHeading){
        double multiplier;
        //double processedInches = -1 * inches;
        telemetry.addData("RawPos", forwardOdo.getCurrentPosition());
        double forwardBackStart = getInchesTraveled(forwardOdo, 0);
        telemetry.addData("Start", forwardBackStart);
        //The variable above gives the position, in inches, that the encoder was in at the start of the function.
        double forwardBackPos = getInchesTraveled(forwardOdo, forwardBackStart);
        telemetry.addData("FirstPos", forwardBackPos);
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("GoStraight", "goal heading is " + targetheading);
        while(forwardBackPos < inches && opModeIsActive()){
            telemetry.addData("Position", forwardBackPos);
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
        telemetry.addData("Inches", getInchesTraveled(forwardOdo, forwardInitialTicks));
        telemetry.update();
    }
    public void goBackward(double power, double inches, double idealHeading){
        double multiplier;
        double forwardBackStart = getInchesTraveled(forwardOdo, 0);
        telemetry.addData("Start", forwardBackStart);
        //The variable above gives the position, in inches, that the encoder was in at the start of the function.
        double forwardBackPos = getInchesTraveled(forwardOdo, forwardBackStart);
        telemetry.addData("FirstPos", forwardBackPos);
        motorFR.setPower(-power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(-power);
        double targetheading = idealHeading;
        RobotLog.aa("GoBackward", "goal heading is " + targetheading);
        while(forwardBackPos > -1 * inches && opModeIsActive()){
            double heading = newGetHeading();
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(-power*multiplier);
                motorBR.setPower(-power*multiplier);
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
    }
    //going to the right is positive ticks, so going to the left will be negative
    public void strafeRight(double power, double inches, double timelimit, double idealHeading){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        double strafeStart = getInchesTraveled(strafeOdo, 0);
        double strafePos = getInchesTraveled(strafeOdo, strafeStart);
        motorFR.setPower(-power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(-power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeRight", "goal heading is " + targetheading);
        while(strafePos < inches && current-startthing < 1000*timelimit && opModeIsActive()){
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
    public void strafeLeft(double power, double inches, double timelimit, double idealHeading){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        double strafeStart = getInchesTraveled(strafeOdo, 0);
        double strafePos = getInchesTraveled(strafeOdo, strafeStart);
        motorFR.setPower(power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("StrafeLeft", "goal heading is " + Double.toString(targetheading));
        while(strafePos > -1 * inches && current - startthing < 1000*timelimit && opModeIsActive()){
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
        motorFR = hardwareMap.get(DcMotor.class, "motor0");
        motorFL = hardwareMap.get(DcMotor.class, "motor1");
        motorBR = hardwareMap.get(DcMotor.class, "motor2");
        motorBL = hardwareMap.get(DcMotor.class, "motor3");
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        strafeOdo = hardwareMap.get(DcMotor.class, "motor0");
        strafeInitialTicks = strafeOdo.getCurrentPosition();
        forwardOdo = hardwareMap.get(DcMotor.class, "motor3");
        forwardInitialTicks = forwardOdo.getCurrentPosition();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
