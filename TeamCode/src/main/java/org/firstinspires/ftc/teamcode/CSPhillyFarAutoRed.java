package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class CSPhillyFarAutoRed extends CSPhillyAuto {
    public void runOpMode(){
        doRun("Red", false);
        goBackward(.35, 3);
        sleep(1000);
    }
}
