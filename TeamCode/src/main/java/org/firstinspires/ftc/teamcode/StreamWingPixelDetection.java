package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class StreamWingPixelDetection extends PPBotCSDF{
    public void runOpMode(){
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        EverythingProcessor p = new EverythingProcessor();
        p.setMode(1);
        VisionPortal portal = VisionPortal.easyCreateWithDefaults(webcam, p);
        portal.resumeStreaming();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Message", "This is for a camera stream that shows the wing pixel processor's results. You need to be in init for that.");
            telemetry.update();
        }
    }
}
