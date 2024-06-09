package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class ServoWithProfiling {
    private static ArrayList<ServoWithProfiling> arr = new ArrayList<>();
    private ServoImplEx s;
    private AnalogInput ana;
    private double currentPos;
    private double initialPos;
    private boolean isMoving = false;
    private double startPosEnc;
    private double endPosEnc;
    private double setTo;
    private double m1;
    private double m2;
    private double b1;
    private double b2;
    private double msForProfile;
    private double percentStartProfiling;
    ElapsedTime t = new ElapsedTime();

    public ServoWithProfiling(ServoImplEx se, AnalogInput a, double m11, double m22, double b11, double b22, double ms, double per) {
        s = se;
        ana = a;
        m1 = m11;
        m2 = m22;
        b1 = b11;
        b2 = b22;
        msForProfile = ms;
        currentPos = ana.getVoltage() / 3.3 * 360;
        initialPos = currentPos;
        percentStartProfiling = per;
        arr.add(this);
    }

    public ServoWithProfiling(String se, String a, double m11, double m22, double b11, double b22, double ms, double per) {
        this(hardwareMap.get(ServoImplEx.class, se), hardwareMap.get(AnalogInput.class, a), m11, m22, b11, b22, ms, per);
    }

    public double servoToEncoderPos(double servoPos) {
        return m1 * servoPos + b1;
    }

    public double encoderToServoPos(double encoder) {
        return m2 * encoder + b2;
    }

    public void updateCurrentPos() {
        currentPos = ana.getVoltage() / 3.3 * 360 - initialPos;
    }

    public double getCurrentPos() {
        return this.currentPos;
    }

    public double getInitialPos() {
        return this.initialPos;
    }
    public void setStartPosEnc(double spe) {
        if (!isMoving) {
            this.startPosEnc = spe;
        }
    }
    public double getStartPosEnc() {
        return this.startPosEnc;
    }
    public void setEndPosEnc(double epe) {
        this.endPosEnc = epe;
        isMoving = true;
    }
    public double getEndPosEnc() {
        return this.endPosEnc;
    }
    public boolean getIsMoving() {
        return isMoving;
    }
    public void setIsMoving(boolean im) {
        isMoving = im;
    }

    public static void updateAllPos() {
        for (int i = 0; i < arr.size(); i++) {
            arr.get(i).updateCurrentPos();
        }
    }

    public void setPosition(double p) {
        s.setPosition(p);
        this.isMoving = false;
        this.setTo = p;
    }

    public void profile() { //also updates isMoving
        isMoving = true;
        if (startPosEnc < endPosEnc) {
            if (currentPos > startPosEnc + (endPosEnc - startPosEnc) * percentStartProfiling) {
                //we want to be profiling
                double idealEncoderPos = startPosEnc + (t.milliseconds()/msForProfile)*(endPosEnc - startPosEnc);
                double idealServoPos = encoderToServoPos(idealEncoderPos);
                s.setPosition(idealServoPos);
                setTo = idealServoPos;
                if (t.milliseconds() > msForProfile || currentPos > endPosEnc) {
                    s.setPosition(encoderToServoPos(endPosEnc));
                    setTo = encoderToServoPos(endPosEnc);
                    isMoving = false;
                }
            } else {
                s.setPosition(encoderToServoPos(endPosEnc));
                setTo = encoderToServoPos(endPosEnc);
                isMoving = true; //we haven't even started profiling yet, so we still need to be moving
                t.reset();
            }
        } else { //startPos > endPos
            if (currentPos < startPosEnc - (startPosEnc - endPosEnc) * percentStartProfiling) {
                //we want to be profiling
                double idealEncoderPos = startPosEnc - (t.milliseconds()/msForProfile)*(startPosEnc - endPosEnc);
                double idealServoPos = encoderToServoPos(idealEncoderPos);
                s.setPosition(idealServoPos);
                setTo = idealServoPos;
                if (t.milliseconds() > msForProfile || currentPos < endPosEnc) {
                    s.setPosition(encoderToServoPos(endPosEnc));
                    setTo = encoderToServoPos(endPosEnc);
                    isMoving = false;
                }
            } else {
                s.setPosition(encoderToServoPos(endPosEnc));
                setTo = encoderToServoPos(endPosEnc);
                isMoving = true; //we haven't even started profiling yet, so we still need to be moving
                t.reset();
            }
        }
    }
}
