package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous
public class StreamRedProp extends LinearOpMode {
    public void runOpMode(){
        VisionPortal portal;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        EverythingProcessor p = new EverythingProcessor();
        p.setMode(EverythingProcessor.ProcessorMode.PROP);
        p.setAlliance(1); //for red
        portal = VisionPortal.easyCreateWithDefaults(webcam, p);
        portal.resumeStreaming();
        while(portal.getCameraState() != VisionPortal.CameraState.STREAMING){
            telemetry.addData("Status", "not ready");
            telemetry.update();
        }
        double leftInitial = p.getLeftVal();
        double centerInitial = p.getCenterVal();
        double rightInitial = p.getRightVal();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Result", p.getResult(leftInitial, centerInitial, rightInitial));
            telemetry.update();
        }
    }
}
