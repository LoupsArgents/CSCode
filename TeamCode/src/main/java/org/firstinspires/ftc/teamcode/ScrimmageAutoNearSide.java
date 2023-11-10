package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class ScrimmageAutoNearSide extends PPBotCSDF {
    //centered lineup: right edge of driverail is on the left edge of the third tab (including the little half-tab on the very edge) from the right of the tile
    //so there are 2 indents to the right of it
    public void runOpMode(){
        initializeHardware();
        processor.setAlliance(1);

        turret.setPosition(turretPos);
        v4b.setPosition(v4bDownPos);
        String result = "Center";
        while(opModeInInit()){
            result = processor.getResult();
            telemetry.addData("Result", result);
            telemetry.update();
        }
        waitForStart();
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
        goStraight(.5, 15, 0.0);
        v4b.setPosition(v4bSlightlyUpPos);
        sleep(200);
        processor.setMode(1);
        if(result.equals("Left")){
            absoluteHeading(.3, 45);
            sleep(200);
            openClaw();
            sleep(500);
            absoluteHeading(.3, 0.0);
        }else if(result.equals("Center")){
            goStraight(.3, 6, 0.0);
            sleep(200);
            goBackward(.3, 2, 0.0);
            sleep(200);
            openClaw();
            sleep(500);
            goBackward(.3, 4, 0.0);
            sleep(100);
        }else if(result.equals("Right")){
            absoluteHeading(.3, -45);
            sleep(200);
            openClaw();
            sleep(500);
            absoluteHeading(.3, 0.0);
            goBackward(.3, 3, 0.0);
        }
        strafeRight(.3, 7, 5, 0.0);
        arm.setTargetPosition(armDownPos);
        v4b.setPosition(v4bDownPos);
        absoluteHeading(.3, -180.0);
        absoluteHeading(.25, -180.0);
        sleep(100);
        if(!result.equals("Right")) {
            goStraight(.3, 6, -180.0);
        }else{
            goStraight(.3, 3, -180.0);
        }
        centerOnClosestStack(processor);
        sleep(500);
        portal.setProcessorEnabled(processor, false);
        portal.setProcessorEnabled(ATProcessor, true);
        goBackward(.5, 6, -180.0);
        absoluteHeading(.3, -90.0);
        absoluteHeading(.2, -90.0);
        goStraight(.5, 14, -90.0);
        strafeLeft(.4, 8, 5, -90.0);
    }
}
