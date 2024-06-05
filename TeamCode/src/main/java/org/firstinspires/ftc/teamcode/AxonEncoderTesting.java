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
public class AxonEncoderTesting extends LinearOpMode {
    /*
    * Servo profiling needs to be coded-- might need more variables passed in to support that.
    * */
    public ServoImplEx axon;
    public AnalogInput axonAna;
    double axonCurrentPosition;
    double axonInitial;
    boolean axonIsMoving = false;
    double startPosEnc;
    double endPosEnc;
    ElapsedTime aTimer = new ElapsedTime();
    boolean aJustStartedProfiling = true;
    /*
    * //AnalogInput armAna = hardwareMap.get(AnalogInput.class, "armAna");
        //armCurrentPosition = armAna.getVoltage() / 3.3 * 360;
        //armInitial = armCurrentPosition;
    * */

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        axon = hardwareMap.get(ServoImplEx.class, "axon");
        axonAna = hardwareMap.get(AnalogInput.class, "axonAna");
        axon.setPosition(0);
        axonCurrentPosition = axonAna.getVoltage() / 3.3 * 360;
        axonInitial = axonCurrentPosition;
        telemetry.addData("status", "initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("axonCurrentPosition", axonCurrentPosition);
            telemetry.addData("axonInitial", axonInitial);
            telemetry.update();
            axonCurrentPosition = axonAna.getVoltage() / 3.3 * 360 - axonInitial;
            if (!axonIsMoving) {
                if (gamepad1.a) {
                    axon.setPosition(0); //339 axon encoder
                    axonIsMoving = true;
                } else if (gamepad1.b) {
                    axon.setPosition(1.0 / 3); //232 axon encoder
                    axonIsMoving = true;
                } else if (gamepad1.y) {
                    axon.setPosition(2.0 / 3); //124 axon encoder
                    axonIsMoving = true;
                } else if (gamepad1.x) {
                    axon.setPosition(1); //17 axon encoder
                    axonIsMoving = true;
                }
            } else {
                axonIsMoving = motionProfile(axon, 500, 0.75, startPosEnc, axonCurrentPosition, endPosEnc);
            }

        }
    }
    public static double aServoPosToEncoder(double servoPos) {
        return -322.2*servoPos + 339.1;
    }
    public static double aEncoderToServoPos(double encoder) {
        return -0.00310365*encoder + 1.05245;
    }
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
    }
}
