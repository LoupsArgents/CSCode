package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class ScrimmageAutoFarSide extends PPBotCSDF {
    public void runOpMode(){
        initializeHardware();
        processor.setAlliance(1);
        closeClaw();
        sleep(500);
        turret.setPosition(turretPos);
        v4b.setPosition(v4bDownPos);
        String result = "Center";
        while(opModeInInit()){
            result = processor.getResult();
            telemetry.addData("Result", result);
            telemetry.update();
        }
        waitForStart();
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
        if(result.equals("Left")){
            absoluteHeading(.3, 45);
            sleep(200);
            openClaw();
        }else if(result.equals("Center")){
            goStraight(.3, 5, 0.0);
            sleep(200);
            goBackward(.3, 1, 0.0);
            sleep(200);
            openClaw();
        }else if(result.equals("Right")){
            absoluteHeading(.3, -45);
            sleep(200);
            openClaw();
        }
        sleep(500);
    }
}