package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class CSPhillyFarAutoBlue extends CSPhillyAuto {
    public void runOpMode(){
        doRun("Blue", false);
        goBackward(.35, 3);
        sleep(1000);
    }
}