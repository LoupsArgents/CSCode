package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

public class CSYorkFarAutoRed extends CSYorkAuto {
    public void runOpMode(){
        doRun("Red", false);
        goBackward(.35, 3);
        sleep(1000);
    }
}
