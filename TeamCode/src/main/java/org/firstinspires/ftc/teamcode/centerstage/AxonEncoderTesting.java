package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class AxonEncoderTesting extends LinearOpMode {
    public ServoWithProfiling a;
    public ServoImplEx axon;
    public AnalogInput axonAna;
    /*
    * //AnalogInput armAna = hardwareMap.get(AnalogInput.class, "armAna");
        //armCurrentPosition = armAna.getVoltage() / 3.3 * 360;
        //armInitial = armCurrentPosition;
    * */

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        axon = hardwareMap.get(ServoImplEx.class, "axon");
        axonAna = hardwareMap.get(AnalogInput.class, "axonAna");
        a = new ServoWithProfiling(axon, axonAna, -322.2, -0.00310365, 339.1, 1.05245, 1000, 0.5);
        a.setPosition(0);
        a.updateCurrentPos();
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
            if (gamepad1.guide || !a.getIsMoving()) {
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
            }
        }
    }
    /*
    public static boolean motionProfile(ServoImplEx servo, double msForProfile, double percentStartProfiling, double startPos, double currentPos, double endPos) {
        //start, current, and end are all encoder values
        //return true if still moving, false if done
        if (startPos < endPos) {
            if (currentPos > startPos + (endPos - startPos) * percentStartProfiling) {
                //we want to be profiling

            } else {
                servo.setPosition(aEncoderToServoPos(endPos));
                return true; //we haven't even started profiling yet
            }
        } else { //startPos > endPos
            if (currentPos < startPos - (startPos - endPos) * percentStartProfiling) {
                //we want to be profiling

            } else {
                servo.setPosition(aEncoderToServoPos(endPos));
                return true; //we haven't even started profiling yet
            }
        }
        return false;
    }*/
}
