package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous
public class StreamBlueProp extends LinearOpMode {
    public void runOpMode(){
        VisionPortal portal;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        EverythingProcessor p = new EverythingProcessor();
        p.setMode(0);
        p.setAlliance(-1); //for blue
        portal = VisionPortal.easyCreateWithDefaults(webcam, p);
        portal.resumeStreaming();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Result", p.getResult());
            telemetry.update();
        }
    }
}
