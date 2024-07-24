package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
//NOT AN OPMODE, DO NOT TRY TO RUN
public class Position {
    private double x;
    private double y;
    private double h;
    public Position(double xPos, double yPos, double heading){
        x = xPos;
        y = yPos;
        h = heading;
    }
    public double getX(){return x;}
    public double getY(){return y;}
    public double getHeading(){return h;}
}