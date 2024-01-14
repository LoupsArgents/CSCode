package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.time.ZonedDateTime;
import java.util.Arrays;

@Autonomous
public class BasicScrimmageAuto extends PPBotCSDF {
    String result = "Center";
    public void runOpMode(){
        initializeHardware();
        processor.setMode(EverythingProcessor.ProcessorMode.PROP);
        processor.setAlliance(1);
        closeClaw();
        sleep(500);
        turret.setPosition(turretPos);
        v4b.setPosition(v4bDownPos);
        waitForStart();
        //this^^^ is all the init stuff that needs to get copied into new programs
    }
    public void doRun(int alliance){ //1 is red, -1 is blue
        closeClaw();
        sleep(500);
        arm.setTargetPosition(armSpikeMarkPos);
        arm.setPower(0.7);
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        while(opModeIsActive() && currentTime - startTime < 3000){
            //result = processor.getResult();
            telemetry.addData("Result", result);
            telemetry.addData("Time", currentTime-startTime);
            telemetry.update();
            currentTime = System.currentTimeMillis();
        }
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("ScrimmageBasicProp " + time);
        double[] thing = processor.getVals();
        RobotLog.aa("LeftCenterRightValues", Arrays.toString(thing));
        arm.setTargetPosition(armSlightlyOffGroundPos);
        sleep(2000);
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
