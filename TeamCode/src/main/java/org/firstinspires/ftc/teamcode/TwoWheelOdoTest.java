package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CSYorkDF;
import org.firstinspires.ftc.teamcode.TwoWheelOdo;
@Autonomous
public class TwoWheelOdoTest extends CSYorkDF {
    TwoWheelOdo odometry;
    public void runOpMode(){
        initializeHardware();
        odometry = new TwoWheelOdo(1892, -1892, (8192/(2*Math.PI)), -(8192/(2*Math.PI)), forwardOdo, strafeOdo, forwardOdo.getCurrentPosition(), strafeOdo.getCurrentPosition(), imu);
        int startForwardTicks = forwardOdo.getCurrentPosition();
        waitForStart();
        modGoStraight(.3, 24, 0.0);
        sleep(1000);
        int endForwardTicks = forwardOdo.getCurrentPosition();
        while(opModeIsActive()){
            odometry.runOdo();
            double xInches = odometry.getX();
            double yInches = odometry.getY();
            telemetry.addData("X", xInches);
            telemetry.addData("Y", yInches);
            telemetry.addData("Difference", endForwardTicks-startForwardTicks);
            telemetry.addData("InchesByEasyMath", ((double)endForwardTicks-(double)startForwardTicks)/1892);
            telemetry.update();
        }
    }
    public void modGoStraight(double power, double inches, double idealHeading){
        double multiplier;
        //double processedInches = -1 * inches;
        int forwardBackStartTicks = forwardOdo.getCurrentPosition();
        telemetry.addData("StartTicks", forwardBackStartTicks);
        int forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        motorFR.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        double targetheading = idealHeading;
        RobotLog.aa("GoStraight", "goal heading is " + targetheading);
        while(newInchesTraveled(forwardBackStartTicks, forwardBackCurrentTicks) < inches && opModeIsActive()){
            odometry.runOdo();
            double xInches = odometry.getX();
            double yInches = odometry.getY();
            telemetry.addData("X", xInches);
            telemetry.addData("Y", yInches);
            liftWithinLoop();
            double heading = newGetHeading();
            //RobotLog.aa("CurrentHeading", String.valueOf(heading));
            if(heading-targetheading<0){  //we need to turn left
                //RobotLog.aa("Going", "Left");
                multiplier = -.1*(heading-targetheading)+1;
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power * multiplier);
                motorBR.setPower(power * multiplier);
            }else if(heading-targetheading>=0){
                //RobotLog.aa("Going", "Right");
                multiplier = .1*(heading-targetheading)+1;
                motorFR.setPower(power);
                motorBR.setPower(power);
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
            }
            forwardBackCurrentTicks = forwardOdo.getCurrentPosition();
        }
        stopMotors();
    }
}
