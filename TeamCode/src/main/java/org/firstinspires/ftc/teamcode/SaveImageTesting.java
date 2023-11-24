package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@TeleOp
public class SaveImageTesting extends LinearOpMode {
    public void runOpMode(){
        VisionPortal portal;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        EverythingProcessor p = new EverythingProcessor();
        p.setMode(0);
        p.setAlliance(1); //for red
        portal = VisionPortal.easyCreateWithDefaults(webcam, p);
        portal.resumeStreaming();
        waitForStart();
        portal.saveNextFrameRaw("testing"); //do I know what I'm doing? absolutely not! is it working? ABSOLUTELY YES!
        while(opModeIsActive()){
            telemetry.addData("Result", p.getResult());
            telemetry.update();
        }
    }
}
