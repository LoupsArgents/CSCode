package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestAutoPickup extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        openClaw();
        processor.setMode(EverythingProcessor.ProcessorMode.PIXEL);
        cameraBar.setPosition(camUsePos);
        waitForStart();
        centerOnClosestStack(processor);
        while(opModeIsActive()){}
    }
}
