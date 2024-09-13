package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OpticalOdoTesting extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        telemetry.addLine("Ready to run");
        telemetry.update();
        waitForStart();
        for(int i = 0; i < 180; i += 15) {
            double x = Math.cos(Math.toRadians(i)) * 12;
            double y = Math.sin(Math.toRadians(i)) * 12;
            moveTo(.5, new Position(x, y, 0.0), true);
            wait(500);
            moveTo(.5, new Position(0, 0, 0.0), true);
            wait(500);
        }
        while(opModeIsActive()){

        }
    }
}
