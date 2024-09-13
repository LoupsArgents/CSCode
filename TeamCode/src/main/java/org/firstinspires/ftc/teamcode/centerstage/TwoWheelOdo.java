package org.firstinspires.ftc.teamcode.centerstage;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.Math;
public class TwoWheelOdo {
    private final double VERTICAL_TICKS_PER_INCH;
    private final double HORIZONTAL_TICKS_PER_INCH;
    private final double VERTICAL_TICKS_PER_RADIAN;
    private final double HORIZONTAL_TICKS_PER_RADIAN;
    private double previousVerticalTicks;
    private double previousHorizontalTicks;
    private double previousHeading;
    private DcMotorEx forwardOdo;
    private DcMotorEx strafeOdo;
    private IMU imu;
    private double x;
    private double y;
    //2pi radians in a rotation -> 1/2pi rotations in a radian
    //ticks per inch are +-1892, ticks per radian are +-8192/(2pi)
    public TwoWheelOdo(double vertTPI, double horTPI, double verTPR, double horTPR, DcMotorEx forward, DcMotorEx strafe, int fInit, int sInit, IMU im){
        VERTICAL_TICKS_PER_INCH = vertTPI;
        HORIZONTAL_TICKS_PER_INCH = horTPI;
        VERTICAL_TICKS_PER_RADIAN = verTPR;
        HORIZONTAL_TICKS_PER_RADIAN = horTPR;
        imu = im;
        forwardOdo = forward;
        strafeOdo = strafe;
    }
    public void runOdo(){
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double deltaVerticalTicks = forwardOdo.getCurrentPosition() - previousVerticalTicks;
        double deltaHorizontalTicks = strafeOdo.getCurrentPosition() - previousHorizontalTicks;
        double deltaHeading = AngleUnit.normalizeRadians(heading - previousHeading);
        double averageHeading = AngleUnit.normalizeRadians(heading + deltaHeading/2);
        RobotLog.aa("AvgHeading", String.valueOf(averageHeading));
        double sin = Math.sin(averageHeading);
        double cos = Math.cos(averageHeading);
        double verticalInches = (deltaVerticalTicks - (deltaHeading * VERTICAL_TICKS_PER_RADIAN))/VERTICAL_TICKS_PER_INCH;
        double horizontalInches = (deltaHorizontalTicks - (deltaHeading * HORIZONTAL_TICKS_PER_RADIAN))/HORIZONTAL_TICKS_PER_INCH;
        double deltaX = cos*verticalInches + sin*horizontalInches;
        double deltaY = sin*verticalInches - cos*horizontalInches;
        x += deltaX;
        y += deltaY;
        previousVerticalTicks = forwardOdo.getCurrentPosition();
        previousHorizontalTicks = strafeOdo.getCurrentPosition();
        previousHeading = heading;
    }
    public double getX(){return x;}
    public double getY(){return y;}
}
