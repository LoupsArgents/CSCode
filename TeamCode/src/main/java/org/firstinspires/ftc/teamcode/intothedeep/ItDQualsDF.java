package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcode.centerstage.SparkFunOTOS;

@Disabled

public class ItDQualsDF extends LinearOpMode {
    Blinker control_Hub;
    IMU imu;
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx motorBL;
    SparkFunOTOS opticalOdo;
    double previousHeading;
    double processedHeading;
    double inchTolerance;
    double degreeTolerance;
    public void runOpMode(){
        initializeHardware();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
    public void goStraight(double power, double inches, double idealHeading){
        double multiplier;
        //double processedInches = -1 * inches;
        double forwardBackStart = opticalOdo.getPosition().y;
        double forwardBackCurrent = opticalOdo.getPosition().y;
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        RobotLog.aa("GoStraight", "goal heading is " + idealHeading);
        while((forwardBackCurrent - forwardBackStart) < inches && opModeIsActive()){
            double heading = newGetHeading();
            //RobotLog.aa("CurrentHeading", String.valueOf(heading));
            if(heading- idealHeading <0){  //we need to turn left
                //RobotLog.aa("Going", "Left");
                multiplier = -.1*(heading- idealHeading)+1;
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
                motorBR.setPower(power * multiplier);
            }else if(heading- idealHeading >=0){
                //RobotLog.aa("Going", "Right");
                multiplier = .1*(heading- idealHeading)+1;
                motorFR.setPower(power);
                motorBR.setPower(power);
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
            }
            forwardBackCurrent = opticalOdo.getPosition().y;
        }
        stopMotors();
    }
    public void goBackward(double power, double inches, double idealHeading){
        double multiplier;
        double forwardBackStart = opticalOdo.getPosition().y;
        double forwardBackCurrent = opticalOdo.getPosition().y;
        motorFR.setPower(-power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(-power);
        RobotLog.aa("GoBackward", "goal heading is " + idealHeading);
        while((forwardBackCurrent - forwardBackStart) > -inches && opModeIsActive()){
            double heading = newGetHeading();
            if(heading- idealHeading < 0){ //turn to the left
                multiplier = -.1*(heading- idealHeading)+1;
                motorFL.setPower(-power*multiplier);
                motorBL.setPower(-power*multiplier);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
            }else if(heading- idealHeading >= 0){ //turn to the right
                multiplier = .1*(heading- idealHeading)+1;
                motorFR.setPower(-power*multiplier);
                motorBR.setPower(-power*multiplier);
                motorFL.setPower(-power);
                motorBL.setPower(-power);
            }
            forwardBackCurrent = opticalOdo.getPosition().y;
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
    public void goStraightForTime(double power, double seconds, double idealHeading){
        double multiplier;
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        RobotLog.aa("GoStraight", "goal heading is " + idealHeading);
        long startTime = System.nanoTime();
        long currentTime = System.nanoTime();
        while((double)(currentTime-startTime)/Math.pow(10, 9) < seconds && opModeIsActive()){
            double heading = newGetHeading();
            //RobotLog.aa("CurrentHeading", String.valueOf(heading));
            if(heading- idealHeading <0){  //we need to turn left
                //RobotLog.aa("Going", "Left");
                multiplier = -.1*(heading- idealHeading)+1;
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
                motorBR.setPower(power * multiplier);
            }else if(heading- idealHeading >=0){
                //RobotLog.aa("Going", "Right");
                multiplier = .1*(heading- idealHeading)+1;
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
        double forwardBackStart = opticalOdo.getPosition().y;
        double forwardBackCurrent = opticalOdo.getPosition().y;
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        RobotLog.aa("GoStraight", "goal heading is " + idealHeading);
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        while(forwardBackCurrent - forwardBackStart < inches && (currentTime-startTime) < timeLimit*1000 && opModeIsActive()){
            double heading = newGetHeading();
            //RobotLog.aa("CurrentHeading", String.valueOf(heading));
            if(heading- idealHeading <0){  //we need to turn left
                //RobotLog.aa("Going", "Left");
                multiplier = -.1*(heading- idealHeading)+1;
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
                motorBR.setPower(power * multiplier);
            }else if(heading- idealHeading >=0){
                //RobotLog.aa("Going", "Right");
                multiplier = .1*(heading- idealHeading)+1;
                motorFR.setPower(power);
                motorBR.setPower(power);
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
            }
            currentTime = System.currentTimeMillis();
            forwardBackCurrent = opticalOdo.getPosition().y;
        }
        stopMotors();
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
            }else if (joyX != 0){
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
            now = System.nanoTime();
        }
    }
    public void setInchTolerance(double i){inchTolerance = i;}
    public void setDegreeTolerance(double d){degreeTolerance = d;}
    public void initializeHardware(){
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFRandForwardEncoder");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFLandStrafeOdo");
        opticalOdo = hardwareMap.get(SparkFunOTOS.class, "optical");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEncoder");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBLandForwardOdo");
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        //configure the optical odo
        opticalOdo.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        opticalOdo.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -0.66, 180);
        opticalOdo.setOffset(offset);
        opticalOdo.setLinearScalar(0.986333);
        opticalOdo.setAngularScalar(0.995088);
        opticalOdo.calibrateImu();
        opticalOdo.resetTracking();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        degreeTolerance = 1;
        inchTolerance = 1;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
