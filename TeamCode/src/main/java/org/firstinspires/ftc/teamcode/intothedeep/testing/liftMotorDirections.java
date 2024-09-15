package org.firstinspires.ftc.teamcode.intothedeep.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class liftMotorDirections extends LinearOpMode {
    double power;
    DcMotorEx lift1;
    DcMotorEx lift2;
    DcMotorEx liftEnc;
    double ticksPerRotation;
    double liftPos;
    double liftInitial;
    public void runOpMode() {
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        liftEnc = hardwareMap.get(DcMotorEx.class, "motorBRandLiftEnc");
        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);

        ticksPerRotation = liftEnc.getMotorType().getTicksPerRev();
        liftInitial = liftEnc.getCurrentPosition()/ticksPerRotation;
        liftPos = liftEnc.getCurrentPosition()/ticksPerRotation - liftInitial;

        waitForStart();

        while(opModeIsActive()) {
            power = gamepad1.left_stick_y;
            liftPos = liftEnc.getCurrentPosition()/ticksPerRotation - liftInitial;
            if (Math.abs(power) < 0.05) {
                power = 0.0;
            }

            telemetry.addData("liftPos", liftPos);
            telemetry.addData("power", power);
            telemetry.update();

            lift1.setPower(power);
            lift2.setPower(power);
        }
    }
}
