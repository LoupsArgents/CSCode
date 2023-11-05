package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OdoTesting extends PPBotCSDF {
    public void runOpMode(){
        initializeHardware();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Forward", forwardOdo.getCurrentPosition());
            telemetry.addData("Strafe", strafeOdo.getCurrentPosition());
            telemetry.update();
        }
    }
}
