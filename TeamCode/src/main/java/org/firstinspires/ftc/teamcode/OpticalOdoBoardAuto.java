package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class OpticalOdoBoardAuto extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        moveTo(.5, 10, 20, 90.0);
        telemetry.addData("Status", "GOT HERE");
        telemetry.update();
    }
}
