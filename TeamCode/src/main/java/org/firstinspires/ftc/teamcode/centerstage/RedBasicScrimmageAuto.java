package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

public class RedBasicScrimmageAuto extends BasicScrimmageAuto {
    public void runOpMode(){
        initializeHardware();
        processor.setMode(EverythingProcessor.ProcessorMode.PROP);
        processor.setAlliance(1);
        turret.setPosition(turretPos);
        v4b.setPosition(v4bDownPos);
        waitForStart();
        doRun(1);
    }
}
