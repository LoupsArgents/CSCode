package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IMUTesting extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        long startTime = System.currentTimeMillis();
        boolean done = false;
        while(opModeIsActive()){
            telemetry.addData("Heading", opticalOdo.getPosition().h);
            telemetry.addData("Done", done);
            telemetry.update();
            if(!done && System.currentTimeMillis() - startTime > 60000){
                opticalOdo.resetTracking();
                done = true;
            }
        }
    }
}
