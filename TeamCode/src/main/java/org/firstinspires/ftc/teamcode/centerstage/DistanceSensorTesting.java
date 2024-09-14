package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
@Disabled
public class DistanceSensorTesting extends CSYorkDF{
    public void runOpMode(){
        initializeHardware();
        waitForStart();
        while(opModeIsActive()){
            //telemetry.addData("Left", leftDistance.getDistance(DistanceUnit.INCH));
            //telemetry.addData("Right", rightDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
