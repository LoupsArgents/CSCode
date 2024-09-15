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
        /*for(int i = 0; i < 180; i += 15) {
            double x = Math.cos(Math.toRadians(i)) * 12;
            double y = Math.sin(Math.toRadians(i)) * 12;
            moveTo(.7, new Position(x, y, 0.0), true);
            wait(500);
            moveTo(.7, new Position(0, 0, 0.0), true);
            wait(500);
        }*///  |\
        //move \|
        moveTo(.5, new Position(0, 10, 0.0), false, false);
        moveTo(.5, new Position(10, 20, 0.0), true);
        sleep(500);
        moveTo(.5, new Position(10, 10, 0.0), false, false);
        moveTo(.5, new Position(0, 0, 0.0), true);
        while(opModeIsActive()){
            telemetry.addData("Status", "Finished");
            telemetry.update();
        }
    }
}
