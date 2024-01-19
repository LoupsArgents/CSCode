package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class CSYorkFarAutoRed extends CSYorkAuto {
    public void runOpMode(){
        doRun("Red", false);
        goBackward(.35, 3);
        sleep(1000);
    }
}
