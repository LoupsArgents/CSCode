package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled

public class CSBoardDoorAutoBlue extends CSPhillyAuto {
    public void runOpMode(){
        doRun("Blue", true, false);
    }
}