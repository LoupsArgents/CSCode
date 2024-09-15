package org.firstinspires.ftc.teamcode.intothedeep.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//next import line is completely made up


@TeleOp
public class MadelynnClawTesting extends LinearOpMode {
    Servo claw;
    double openPos = 0.745;
    double closePos = 0.625;
    double currentPos;
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("currentPos", currentPos);
            telemetry.update();
            if (gamepad1.right_trigger > 0.05 && currentPos != closePos) {
                claw.setPosition(closePos);
                currentPos = closePos;
            }
            if (gamepad1.left_trigger > 0.05 && currentPos != openPos) {
                claw.setPosition(openPos);
                currentPos = openPos;
            }
        }
    }
}
