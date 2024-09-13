package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp
public class OdoTesting extends PPBotCSDF {
    public void runOpMode(){
        initializeHardware();
        waitForStart();
        int strafeStartTicks = strafeOdo.getCurrentPosition();
        int strafeCurrentTicks;
        double inchesTraveled;
        //strafeLeft(.35, 5, 5, 0.0);
        //strafeRight(.35, 5, 5, 0.0);
        //strafeLeft(.35, 5, 5, 0.0);
        while(opModeIsActive()){
            telemetry.addData("Forward", forwardOdo.getCurrentPosition());
            telemetry.addData("Strafe", strafeOdo.getCurrentPosition());
            strafeCurrentTicks = strafeOdo.getCurrentPosition();
            inchesTraveled = newInchesTraveled(strafeStartTicks, strafeCurrentTicks);
            RobotLog.aa("Ticks", String.valueOf(strafeCurrentTicks));
            RobotLog.aa("Inches", String.valueOf(inchesTraveled));
            telemetry.update();
        }
    }
}
