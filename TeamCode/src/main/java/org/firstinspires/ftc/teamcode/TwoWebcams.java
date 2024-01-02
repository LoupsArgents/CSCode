package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous

public class TwoWebcams extends LinearOpMode {
    public void runOpMode(){
        WebcamName firstCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        WebcamName secondCam = hardwareMap.get(WebcamName.class, "Webcam 2");
        EverythingProcessor p = new EverythingProcessor();
        p.setMode(1);
        CameraName webcam = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(firstCam, secondCam);
        VisionPortal myVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(p)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
        waitForStart();
        while(opModeIsActive()) {
            myVisionPortal.setActiveCamera(firstCam);
            telemetry.addData("FirstCamPos", p.getClosestPixelPos());
            myVisionPortal.setActiveCamera(secondCam);
            long startTime = System.currentTimeMillis();
            long nowTime = System.currentTimeMillis();
            while(nowTime - startTime < 1000 && opModeIsActive()){
                nowTime = System.currentTimeMillis();
            }
            telemetry.addData("SecondCamPos", p.getClosestPixelPos());
            telemetry.update();
            startTime = System.currentTimeMillis();
            nowTime = System.currentTimeMillis();
            while(nowTime - startTime < 1000 && opModeIsActive()){
                nowTime = System.currentTimeMillis();
            }
        }
    }
}