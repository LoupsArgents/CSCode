package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class StreamWingPixelDetection extends CSYorkDF{
    public void runOpMode(){
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        EverythingProcessor p = new EverythingProcessor();
        p.setMode(EverythingProcessor.ProcessorMode.PIXEL);
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 360))
                .addProcessor(p)
                .enableLiveView(true)
                .build();
        portal.resumeStreaming();
        waitForStart();
        while(opModeIsActive()){
            if(!p.updates.equals("")) {
                telemetry.addLine(p.updates);
                //telemetry.addData("Message", "This is for a camera stream that shows the wing pixel processor's results. You need to be in init for that.");
                telemetry.update();
            }
        }
    }
}
