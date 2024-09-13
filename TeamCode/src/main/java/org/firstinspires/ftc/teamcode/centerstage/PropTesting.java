package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.vision.VisionPortal;

import java.time.ZonedDateTime;

@Autonomous
public class PropTesting extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        processor.setAlliance(-1);
        processor.setMode(EverythingProcessor.ProcessorMode.PROP);
        double camLeftInitial = 0.0;
        double camCenterInitial = 0.0;
        double camRightInitial = 0.0;
        cameraBar.setPosition(camUsePos);
        boolean doneTheThing = false;
        long timeStreamingDetected = 0;
        boolean streaming = false;
        while(opModeInInit()) {
            if(portal.getCameraState() == VisionPortal.CameraState.STREAMING && !streaming){
                timeStreamingDetected = System.currentTimeMillis();
                streaming = true;
            }
            long nowTime = System.currentTimeMillis();
            if (!doneTheThing && streaming && (nowTime - timeStreamingDetected > 1000)) {
                ZonedDateTime dt = ZonedDateTime.now();
                String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
                portal.saveNextFrameRaw("PropTestingInitial " + time);
                camLeftInitial = processor.getLeftVal();
                camCenterInitial = processor.getCenterVal();
                camRightInitial = processor.getRightVal();

                RobotLog.aa("CamLeftInitial", String.valueOf(camLeftInitial));
                RobotLog.aa("CamRightInitial", String.valueOf(camRightInitial));
                RobotLog.aa("CamCenterInitial", String.valueOf(camCenterInitial));
                doneTheThing = true;
            }
            if(doneTheThing){
                telemetry.addData("Status", "Now you can put the prop down");
                telemetry.addData("CamLeftInitial", camLeftInitial);
                telemetry.addData("CamCenterInitial", camCenterInitial);
                telemetry.addData("CamRightInitial", camRightInitial);
            }
            telemetry.update();
        }
        waitForStart();
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("PropTestingFinal " + time);
        String result = processor.getResult(camLeftInitial, camCenterInitial, camRightInitial);
        telemetry.addData("Status", "Now you can put the prop down");
        telemetry.addData("CamLeftInitial", camLeftInitial);
        telemetry.addData("CamCenterInitial", camCenterInitial);
        telemetry.addData("CamRightInitial", camRightInitial);
        telemetry.addData("Result", result);
        telemetry.update();
        while(opModeIsActive()){

        }
    }
}
