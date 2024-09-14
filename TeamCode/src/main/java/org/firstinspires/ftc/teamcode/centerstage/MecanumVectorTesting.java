package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class MecanumVectorTesting extends CSYorkDF {
    public void runOpMode(){
        initializeHardware();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        double startX = opticalOdo.getPosition().x;
        double startY = opticalOdo.getPosition().y;
        motorFR.setPower(.5);
        motorBL.setPower(.5);
        sleep(3000);
        stopMotors();
        telemetry.addData("x", opticalOdo.getPosition().x-startX);
        telemetry.addData("y", opticalOdo.getPosition().y-startY);
        telemetry.update();
        while(opModeIsActive()){}
    }
}
