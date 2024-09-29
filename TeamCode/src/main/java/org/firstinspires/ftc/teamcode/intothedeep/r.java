package org.firstinspires.ftc.teamcode.intothedeep;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class r {
    //declare motors, servos, and sensors up here as static variables (make them public for simplicity?)
    //also have positions as static variables (have the values here)
    static ElapsedTime scheduleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static ArrayList<Command> commands = new ArrayList<Command>();
    public static void init() {
        //initialize all of the motors/servos/etc. here
        scheduleTimer.reset();
    }
    public static void doCommands() { //should get called every loop
        for (int i = commands.size() - 1; i >= 0; i--) {
            if (commands.get(i).getAbsTimeStart() >= scheduleTimer.milliseconds()) {
                commands.get(i).adjust();
            }
            if (commands.get(i).getIsDone()) {
                commands.remove(i);
            }
        }
    }
    public static void addCommand(Command c) {
        c.setAbsTimeStart(c.getMS() + scheduleTimer.milliseconds());
        commands.add(c);
    }
    public static void addCommand(Servo ss, double ip, double msTime) {
        addCommand(new ServoCommand(ss, ip, msTime));
    }
    public static void addCommand(DcMotorEx mm, double ip, double msTime, double kp, double initial, double tpr, double stall, double tol) {
        addCommand(new MotorPosCommand(mm, ip, msTime, kp, initial, tpr, stall, tol));
    }
    public static void addCommand(DcMotorEx mm, double msTime, double power) {
        addCommand(new MotorPowCommand(mm, msTime, power));
    }

}