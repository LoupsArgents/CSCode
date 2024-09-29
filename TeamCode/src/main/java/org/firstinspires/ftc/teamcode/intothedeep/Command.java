package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//what are we using: servo or motor? (decided by what subclass of this is used)
//what position do we want it going to?
//how do we adjust it every loop to get it to this ideal position? (going to depend on subclass)
//what absolute time should this happen
public class Command {
    private double idealPos;
    private double ms; //ms after start of all of these commands
    private double absTimeStart;
    private boolean isDone = false;
    public Command (double ip, double msTime) {
        this.idealPos = ip;
        this.ms = msTime;
    }
    public void adjust() {}
    public double getIdealPos() {return this.idealPos;}
    public double getMS() {return this.ms;}
    public double getAbsTimeStart() {return this.absTimeStart;}
    public void setAbsTimeStart(double a) {this.absTimeStart = a;}
    public boolean getIsDone() {return this.isDone;}
    public void setIsDone(boolean id) {this.isDone = id;}
}
class ServoCommand extends Command {
    Servo s;

    public ServoCommand (Servo ss, double ip, double msTime) {
        super(ip, msTime);
        this.s = ss;
    }
    public void adjust() {
        s.setPosition(super.getIdealPos());
        super.setIsDone(true);
    }
}
class MotorCommand extends Command {
    DcMotorEx m;
    double kp;
    double initial;
    double ticksPerRotation;
    double stallPower;
    double tolerance;
    public MotorCommand(DcMotorEx mm, double ip, double msTime, double kp, double initial, double tpr, double stall, double tol) {
        super(ip, msTime);
        this.m = mm;
        this.kp = kp;
        this.initial = initial;
        this.ticksPerRotation = tpr;
        this.stallPower = stall;
        this.tolerance = tol;
    }
    public void adjust() {
        double current = m.getCurrentPosition()/ticksPerRotation - initial;
        double error = super.getIdealPos() - current;
        m.setPower(kp*error);
        if (Math.abs(error) < tolerance) {
            super.setIsDone(true);
            m.setPower(stallPower);
        }
    }
}
