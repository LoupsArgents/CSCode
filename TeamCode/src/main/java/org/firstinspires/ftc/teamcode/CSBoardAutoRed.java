package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class CSBoardAutoRed extends CSPhillyAuto {
    public void runOpMode(){
        doRun("Red", true, false);
    }
}