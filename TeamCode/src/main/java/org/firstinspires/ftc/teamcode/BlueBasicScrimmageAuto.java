package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueBasicScrimmageAuto extends BasicScrimmageAuto {
    public void runOpMode(){
        initializeHardware();
        processor.setMode(0);
        processor.setAlliance(-1);
        turret.setPosition(turretPos);
        v4b.setPosition(v4bDownPos);
        waitForStart();
        doRun(-1);
    }
}
