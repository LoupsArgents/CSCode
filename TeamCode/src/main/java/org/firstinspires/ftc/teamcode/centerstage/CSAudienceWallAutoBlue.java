package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled

public class CSAudienceWallAutoBlue extends CSPhillyAuto {
    public void runOpMode(){
        doRun("Blue", false, true);
    }
}
