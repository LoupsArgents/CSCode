package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OpticalOdoTracking extends LinearOpMode {
    SparkFunOTOS opticalOdo;
    public void runOpMode(){
        opticalOdo = hardwareMap.get(SparkFunOTOS.class, "optical");
        opticalOdo.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        opticalOdo.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -0.66, 180);
        opticalOdo.setOffset(offset);
        opticalOdo.setLinearScalar(0.986333);
        opticalOdo.setAngularScalar(0.995088);
        opticalOdo.calibrateImu();
        opticalOdo.resetTracking();
        waitForStart();
        while(opModeIsActive()){
            SparkFunOTOS.Pose2D pos = opticalOdo.getPosition();
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("heading", pos.h);
            telemetry.update();
        }
    }
}
