package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled

public class CSAudienceWallAutoRed extends CSPhillyAuto {
    public void runOpMode(){
        doRun("Red", false, true);
    }
}
