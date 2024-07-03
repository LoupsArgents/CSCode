package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class OpticalOdoBoardAuto extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        waitForStart();
        moveTo(.5, 2, 4, 90.0);
        sleep(1000);
        moveTo(.5, -4, 2, -90.0);
        sleep(1000);
        moveTo(.5, 2, -6, 0.0);
        sleep(1000);
    }
}
