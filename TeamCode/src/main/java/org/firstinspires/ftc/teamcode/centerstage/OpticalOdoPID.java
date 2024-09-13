package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
public class OpticalOdoPID extends CSYorkDF {
    SparkFunOTOS opticalOdo;
    public void runOpMode(){
        initializeHardware();
        opticalOdo = hardwareMap.get(SparkFunOTOS.class, "optical");
        //configure the odo
        opticalOdo.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        opticalOdo.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -0.66, 180);
        opticalOdo.setOffset(offset);
        opticalOdo.setLinearScalar(0.986333);
        opticalOdo.setAngularScalar(0.995088);
        opticalOdo.calibrateImu();
        opticalOdo.resetTracking();
        telemetry.addLine("Ready to run");
        telemetry.update();
        waitForStart();
        double startX = opticalOdo.getPosition().x;
        double startY = opticalOdo.getPosition().y;
        while(opModeIsActive()){
            placeAndHeading(startX+10, startY+10, 45.0, .5, .25, 1);
        }
    }
    public boolean placeAndHeading(double x, double y, double idealHeading, double powerMult, double inTol, double degTol) {
        processedHeading = newGetHeading();
        double rxConst = 3; //was 6
        double moveConst = 1; //maybe needs editing
        double currentX = opticalOdo.getPosition().x;
        double currentY = opticalOdo.getPosition().y;
        telemetry.addData("currentX, currentY", currentX + ", " + currentY);
        double xDifference = currentX - x;
        double yDifference = currentY - y;
        RobotLog.aa("xDiff_yDiff", xDifference + ", " + yDifference);
        double l = Math.sqrt(xDifference*xDifference + yDifference*yDifference); // diagonal error
        telemetry.addData("Diagonal error", l);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //x = fake joystick left left/right (strafe)
        double joyX = -1 * xDifference / l;
        //telemetry.addData("joyX", joyX);
        //y = fake joystick left up/down (move)
        double joyY = -1 * yDifference / l;
        //telemetry.addData("joyY", joyY);
        if (l < inTol) {
            joyX = 0;
            joyY = 0;
        }
        if (l < 5) {
            moveConst = l/5;
            joyX *= moveConst;
            joyY *= moveConst;
        }
        //rx = fake joystick right left/right aka turning
        double rx = 0;
        double rotError = Math.abs(processedHeading % 360 - idealHeading);
        if (l < inTol && rotError < degTol) {
            //if we actually don't need to do anything
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
            return true;
        }
        double sign = rotError/(processedHeading % 360 - idealHeading);
        if (rotError > 45) {rxConst = 90;}
        if (rotError < degTol) {
            rx = 0;
        } else if (sign == -1) {
            //turning left
            rx = -1 * (rxConst * 180) * rotError;
        } else if (sign == 1) {
            //turning right
            rx = (rxConst/180) * rotError;
        }
        if (rx < -1) {rx = -1;}
        if (rx > 1) {rx = 1;}
        //telemetry.addData("rX", rx);

        double rotX = joyX * Math.cos(-botHeading) - joyY * Math.sin(-botHeading);
        double rotY = joyX * Math.sin(-botHeading) + joyY * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        motorFL.setPower(powerMult * frontLeftPower);
        motorBL.setPower(powerMult * backLeftPower);
        motorFR.setPower(powerMult * frontRightPower);
        motorBR.setPower(powerMult * backRightPower);
        RobotLog.aa("FL_BL_FR_BR", (powerMult*frontLeftPower)+ ", " + (powerMult*backLeftPower) + ", " + (powerMult*frontRightPower) + ", " + (powerMult+backRightPower));
        telemetry.update();
        return false;
    }
    public void moveTo(double power, double x, double y, double heading){
        while(placeAndHeading(x, y, heading, power, .25, .5)){}
    }
}
