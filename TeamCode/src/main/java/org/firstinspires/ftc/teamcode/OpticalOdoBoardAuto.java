package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class OpticalOdoBoardAuto extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        waitForStart();
        moveTo(.5, 10, 20, 90.0);
        sleep(1000);
        //moveTo(.5, -20, 10, -90.0);
        //sleep(1000);
        //moveTo(.5, 10, -30, 0.0);
        sleep(1000);
    }
}
