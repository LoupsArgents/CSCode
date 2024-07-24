package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous
public class OpticalOdoBoardAuto extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        moveTo(.5, new Position(10, 20, 90.0), false); //was heading 90
        moveTo(.5, new Position(5, 10, 150.0), false); //was heading 150
        moveTo(.5, new Position(-15, -30, 0.0), true);
        telemetry.addData("Status", "GOT HERE");
        RobotLog.aa("Status", "DONE!! WE GOT THERE");
        telemetry.update();
        while(opModeIsActive()){
            SparkFunOTOS.Pose2D pos = opticalOdo.getPosition();
            telemetry.addData("CurrentX", pos.x);
            telemetry.addData("CurrentY", pos.y);
            telemetry.addData("HubIMU", newGetHeading());
            telemetry.addData("OpticalIMU", pos.h);
            telemetry.update();
        }
    }
}
