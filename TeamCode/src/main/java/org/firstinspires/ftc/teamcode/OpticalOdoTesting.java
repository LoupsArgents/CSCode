package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
public class OpticalOdoTesting extends CSYorkDF {
    SparkFunOTOS opticalOdo;
    public void runOpMode(){
        initializeHardware();
        opticalOdo = hardwareMap.get(SparkFunOTOS.class, "optical");
        //configure the odo
        opticalOdo.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        opticalOdo.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -0.66, 180);
        opticalOdo.setOffset(offset);
        opticalOdo.setLinearScalar(0.986333);
        opticalOdo.setAngularScalar(0.995088);
        opticalOdo.calibrateImu();
        opticalOdo.resetTracking();
        telemetry.addLine("Ready to run");
        telemetry.update();
        waitForStart();
        int forwardStartTicks = forwardOdo.getCurrentPosition();
        goStraight(.5, 40, 0.0);
        while(opModeIsActive()){
            telemetry.addData("Angle", opticalOdo.getPosition().h);
            telemetry.addData("Forward", opticalOdo.getPosition().y);
            telemetry.addLine("Wheels say " + newInchesTraveled(forwardStartTicks, forwardOdo.getCurrentPosition()));
            telemetry.update();
        }
    }
}
