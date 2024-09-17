package org.firstinspires.ftc.teamcode.intothedeep.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp
public class SwyftDistanceTesting extends LinearOpMode {
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AnalogInput ranger = hardwareMap.get(AnalogInput.class, "distanceSensor");
        Rev2mDistanceSensor distanceLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");
        waitForStart();
        ArrayList<Double> swyftAverageList = new ArrayList<>();
        ArrayList<Double> revAverageList = new ArrayList<>();
        while(opModeIsActive()){
            //20 degree mode code
            double inches = (ranger.getVoltage()*48.7)-4.9;
            swyftAverageList.add(inches);
            revAverageList.add(distanceLeft.getDistance(DistanceUnit.INCH));
            if(swyftAverageList.size() == 6){
                swyftAverageList.remove(0);
            }
            if(revAverageList.size() == 6){
                revAverageList.remove(0);
            }
            double revAvg = 0.0;
            double swyftAvg = 0.0;
            if(swyftAverageList.size() == 5){
                for(double d : swyftAverageList){swyftAvg += d;}
                swyftAvg /= 5;
            }
            if(revAverageList.size() == 5){
                for(double d : revAverageList){revAvg += d;}
                revAvg /= 5;
            }
            telemetry.addData("Swyft", swyftAvg);
            telemetry.addData("REV2m",revAvg);
            telemetry.update();
        }
    }
}
