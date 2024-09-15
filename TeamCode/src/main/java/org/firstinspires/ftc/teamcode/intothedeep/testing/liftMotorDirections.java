package org.firstinspires.ftc.teamcode.intothedeep.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class liftMotorDirections extends LinearOpMode {
    double power;
    DcMotorEx motor;
    String motorName;
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            power = gamepad1.left_stick_y;
            if (Math.abs(power) < 0.05) {
                power = 0.0;
            }
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}
