package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ArmCSProfiling extends LinearOpMode {
    public ServoWithProfiling a;
    public ServoImplEx arm;
    public AnalogInput armAna;
    /*
    * //AnalogInput armAna = hardwareMap.get(AnalogInput.class, "armAna");
        //armCurrentPosition = armAna.getVoltage() / 3.3 * 360;
        //armInitial = armCurrentPosition;
    * */

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(ServoImplEx.class, "arm3");
        armAna = hardwareMap.get(AnalogInput.class, "armAna");
        a = new ServoWithProfiling(arm, armAna, -322.2, -0.00310365, 339.1, 1.05245, 2000, 0.5);
        //a.setPosition(0);
        //a.updateCurrentPos();
        telemetry.addData("status", "initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            ServoWithProfiling.updateAllPos();
            telemetry.addData("axonCurrentPosition", a.getCurrentPos());
            telemetry.addData("axonInitial", a.getInitialPos());
            telemetry.addData("axonStartPos", a.getStartPosEnc());
            telemetry.addData("axonEndPos", a.getEndPosEnc());
            telemetry.addData("servoEndPos", a.encoderToServoPos(a.getEndPosEnc()));
            telemetry.addData("Timer (ms)", a.getMS());
            telemetry.addData("isMoving", a.getIsMoving());
            telemetry.update();
            /*if (gamepad1.guide || !a.getIsMoving()) {
                if (gamepad1.a) {
                    a.setStartPosEnc(a.getCurrentPos());
                    a.setEndPosServo(0.0);
                    a.setIsMoving(true);
                } else if (gamepad1.b) {
                    telemetry.addData("this runs", "");
                    a.setStartPosEnc(a.getCurrentPos());
                    a.setEndPosServo(1.0/3);
                    a.setIsMoving(true);
                } else if (gamepad1.y) {
                    a.setStartPosEnc(a.getCurrentPos());
                    a.setEndPosServo(2.0/3);
                    a.setIsMoving(true);
                } else if (gamepad1.x) {
                    a.setStartPosEnc(a.getCurrentPos());
                    a.setEndPosServo(1.0);
                    a.setIsMoving(true);
                }
            }
            if (a.getIsMoving()) {
                a.profile();
            }*/
        }
    }
}
