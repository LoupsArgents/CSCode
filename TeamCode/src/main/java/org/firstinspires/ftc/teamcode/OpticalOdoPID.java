package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp
public class OpticalOdoPID extends CSYorkDF {
    SparkFunOTOS opticalOdo;
    public void runOpMode(){
        initializeHardware();
        opticalOdo = hardwareMap.get(SparkFunOTOS.class, "optical");
        //configure the odo
        opticalOdo.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
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
        double startX = opticalOdo.getPosition().x/100;
        double startY = opticalOdo.getPosition().y/100;
        while(opModeIsActive()){
            placeAndHeading(startX+10, startY+10, 0.0, .5, 1, 5);
        }
    }
    public boolean placeAndHeading(double x, double y, double idealHeading, double powerMult, double cmTol, double degTol) {
        processedHeading = newGetHeading();
        double rxConst = 6;
        double moveConst = 1;
        double currentX = opticalOdo.getPosition().x/100;
        double currentY = opticalOdo.getPosition().y/100;
        telemetry.addData("currentX, currentY", currentX + ", " + currentY);
        double xDifference = currentX - x;
        double yDifference = currentY - y;
        double l = Math.sqrt(xDifference*xDifference + yDifference*yDifference); // diagonal error
        telemetry.addData("Diagonal error", l);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //x = fake joystick left left/right (strafe)
        double joyX = -1 * xDifference / l;
        //telemetry.addData("joyX", joyX);
        //y = fake joystick left up/down (move)
        double joyY = -1 * yDifference / l;
        //telemetry.addData("joyY", joyY);
        if (l < cmTol) {
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
        if (l < cmTol && rotError < degTol) {
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
        telemetry.update();
        return false;
    }
}
