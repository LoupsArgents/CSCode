package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

public class CSYorkFarAutoBlue extends CSYorkAuto{
    public void runOpMode(){
        doRun("Blue", false);
        goBackward(.35, 3);
        sleep(1000);
    }
}
