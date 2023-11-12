package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;

@Disabled
public class ScrimmageAutoNearSide extends PPBotCSDF {
    //centered lineup: right edge of driverail is on the left edge of the third tab (including the little half-tab on the very edge) from the right of the tile
    //so there are 2 indents to the right of it
    String result = "Center";
    public void runOpMode(){
        initializeHardware();
        processor.setAlliance(1);
        turret.setPosition(turretPos);
        v4b.setPosition(v4bDownPos);
        while(opModeInInit()){
            result = processor.getResult();
            telemetry.addData("Result", result);
            telemetry.update();
        }
        waitForStart();

    }
    public void doRun(int alliance){ //red is 1, blue is -1
        closeClaw();
        sleep(500);
        arm.setTargetPosition(armSpikeMarkPos);
        arm.setPower(0.7);
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        while(opModeIsActive() && currentTime - startTime < 3000){
            result = processor.getResult();
            telemetry.addData("Result", result);
            telemetry.addData("Time", currentTime-startTime);
            telemetry.update();
            currentTime = System.currentTimeMillis();
        }
        arm.setTargetPosition(armSlightlyOffGroundPos);
        goStraight(.5, 16, 0.0);
        v4b.setPosition(v4bSlightlyUpPos);
        sleep(200);
        processor.setMode(1);
        processor.setAlliance(alliance);
        if(result.equals("Left")){
            absoluteHeading(.3, 50); //used to be 45
            sleep(200);
            goStraight(.3, .5, 50.0);
            sleep(200);
            openClaw();
            sleep(500);
            goBackward(.3, .5, 50.0);
            absoluteHeading(.3, 0.0);
            goBackward(.3, 4, 0.0);
        }else if(result.equals("Center")){
            goStraight(.3, 5, 0.0);
            sleep(200);
            goBackward(.3, 2, 0.0);
            sleep(200);
            openClaw();
            sleep(500);
            goBackward(.3, 4, 0.0);
            sleep(100);
        }else if(result.equals("Right")){
            absoluteHeading(.3, -50); //used to be -45
            sleep(200);
            goStraight(.3, .5, -50.0);
            sleep(200);
            openClaw();
            sleep(500);
            goBackward(.3, .5, -50.0);
            absoluteHeading(.3, 0.0);
            goBackward(.3, 4, 0.0);
        }
        if(alliance == 1) {
            strafeRight(.3, 7, 5, 0.0);
        }else{
            strafeLeft(.3, 7, 5, 0.0);
        }
        arm.setTargetPosition(armDownPos);
        wrist.setPosition(wristDownPos);
        absoluteHeading(.3, -180.0*alliance);
        v4b.setPosition(v4bDownPos);
        absoluteHeading(.25, -180.0*alliance);
        sleep(100);
        wrist.setPosition(wristDownPos);
        if(!result.equals("Right")) {
            goStraight(.3, 6, -180.0*alliance);
        }else{
            goStraight(.3, 4.5, -180.0*alliance);
        }
        centerOnClosestStack(processor);
        sleep(500);
        portal.setProcessorEnabled(processor, false);
        portal.setProcessorEnabled(ATProcessor, true);
        goBackward(.5, 6, -180.0*alliance);
        absoluteHeading(.3, -90.0*alliance);
        absoluteHeading(.2, -90.0*alliance);
        goStraight(.5, 14, -90.0*alliance);
        arm.setTargetPosition(armSpikeMarkPos);
        v4b.setPosition(v4bDownPos-.15);
        wrist.setPosition(wristDownPos);
        if(alliance == 1) {
            strafeLeft(.4, 10.5, 5, -90.0 * alliance);
        }else{
            strafeRight(.4, 10.5, 5, -90.0*alliance);
        }
        sleep(1500);
        double[] dist = getAprilTagDist(result);
        telemetry.addData("Distances", Arrays.toString(dist));
        telemetry.update();
        arm.setTargetPosition(armBackdropDeliveryPos);
        v4b.setPosition(v4bDownPos);
        if(dist[0] < 0) {
            strafeRight(.3, Math.abs(dist[0]), 5, -90.0*alliance);
        }else{
            strafeLeft(.3, Math.abs(dist[0]), 5, -90.0*alliance);
        }
        goStraight(.3, dist[1]-15, -90.0*alliance);
        openClaw();
        sleep(500);
        goBackward(.4, 4, -90.0*alliance);
        RobotLog.aa("Position", "First place");
        arm.setTargetPosition(armDownPos);
        v4b.setPosition(v4bDownPos);
        if(alliance == 1) {
            strafeRight(.4, 20, 5, -90.0*alliance); //y u no strafe. Y U NO STRAAAAAFE ohhhhh
            //asdflkjero;ijdfklcxvwe;oijfdv I was giving it a time limit of -90 seconds. ioer;jfslk;fhasjakfdhglkesjfdlsa
            //leaving these comments here for posterity because AAAAAAAAAASDFGHJKL
        }else{
            strafeLeft(.4, 20, 5, -90.0*alliance);
        }
        RobotLog.aa("Position", "Second place");
    }
}