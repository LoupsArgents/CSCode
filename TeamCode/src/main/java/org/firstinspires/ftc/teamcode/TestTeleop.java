package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
//next import line is completely made up
import com.qualcomm.hardware.lynx.LynxNackException;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Queue;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ServoImplEx; // for Axon
import com.qualcomm.robotcore.hardware.PwmControl; // for Axon


@TeleOp
public class TestTeleop extends LinearOpMode {
    public AnalogInput ultra;
    double distance1;
    double distance2;
    double vcc = 3.3;

    public void runOpMode() {

        ultra = hardwareMap.analogInput.get("ultrasonic");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ArrayList<Double> movingAvgList = new ArrayList<Double>();

        waitForStart();

        while (opModeIsActive()) {
            //= [Vobserved / ((Vcc/1024) * 6)] - 300 is the formula on the maxbotix website
            double volt = ultra.getVoltage();
            distance2 = 2*((volt/(vcc/1024)) * 6 - 300); //in mm???
            //distance2 = (volt / ((vcc/1024) * 6)) - 300; their formula-- consistently returns negative values :(
            RobotLog.aa("distance", Double.toString(distance2));
            telemetry.addData("distance2", distance2);
            telemetry.update();
        }
    }



    public double getAvg(ArrayList<Double> readings) {
        double returnAvg = 0;
        for (int i = 0; i < readings.size(); i++) {
            returnAvg += readings.get(i);
        }
        returnAvg /= readings.size();
        return returnAvg;
    }
    /*
    * //= [Vobserved / ((Vcc/1024) * 6)] - 300 is the formula on the maxbotix website
            double volt = ultra.getVoltage();
            distance2 = 2*((volt/(vcc/1024)) * 6 - 300); //in cm!!!
            //distance = ultra.getVoltage();
            //distance = (0.206866*(Math.sqrt((ultra.getVoltage() + 3659.32)*0.06786))) + -3.11436;
            movingAvgList.add(distance2);
            if (movingAvgList.size() > 5) {
                movingAvgList.remove(0);
            }
            //RobotLog.aa("moving average", Double.toString(getAvg(movingAvgList)));
            RobotLog.aa("distance", Double.toString(distance2));
            //telemetry.addData("distance1", distance1);
            telemetry.addData("distance2", distance2);
            telemetry.addData("movingAvg", getAvg(movingAvgList));
            telemetry.update();
    * */

}
