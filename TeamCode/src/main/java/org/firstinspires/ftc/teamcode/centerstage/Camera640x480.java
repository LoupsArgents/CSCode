package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@TeleOp
@Disabled
public class Camera640x480 extends LinearOpMode {
    public void runOpMode(){
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal portal = VisionPortal.easyCreateWithDefaults(webcam);
        portal.resumeStreaming();
        waitForStart();
        while(opModeIsActive()){
            continue;
        }
    }
}
