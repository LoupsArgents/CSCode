package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@TeleOp
public class Camera640x360 extends LinearOpMode {
    public void runOpMode(){
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 360))
                .enableLiveView(true)
                .build();
        portal.resumeStreaming();
        waitForStart();
        while(opModeIsActive()){
            continue;
        }
    }
}
