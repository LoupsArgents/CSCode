package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class UltraInchTesting extends CSYorkDF{
    public void runOpMode(){
        initializeHardware();
        waitForStart();
        armAboveStack();
        wrist.setPosition(wristAboveStackPos);
        //soooo 6.5 inches ultrasonic-to-wall is probably workable
        while(opModeIsActive()){
            telemetry.addData("UltraDistance", frontRightUltraDistance());
            telemetry.addData("ClawLeft", clawLeftSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("ClawRight", clawRightSensor.getDistance(DistanceUnit.INCH));
            // < 2.2 inches should mean that there's a pixel under there (with this .72 arm position)
            // now, how far from the wall do we need to be for that? hopefully the claw guards won't scrape the wall
            telemetry.update();
        }
    }
}
