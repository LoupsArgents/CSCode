/*
Copyright 2021 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp
@Disabled

public class RI3DTeleop extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private Gyroscope imu;
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private Servo bucket;
    private Servo arm;
    private DcMotor lift;
    double drivePower;
    double armDownPos = 0.5;
    double armUpPos = 0.6;
    /*
    private double bucketOpenPos;
    private double bucketClosePos;
     */
    private static double ticksPerRotation;
    static double liftInitial;
    private DcMotor intake;
    private double liftPos;
    private double currentVoltage;
    private double liftPower = 0;

    //@Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorFR = hardwareMap.get(DcMotor.class, "motor0");
        motorFL = hardwareMap.get(DcMotor.class, "motor1");
        motorBR = hardwareMap.get(DcMotor.class, "motor2");
        motorBL = hardwareMap.get(DcMotor.class, "motor3");
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(Servo.class, "arm");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bucket = hardwareMap.get(Servo.class, "bucket");
        drivePower = 0.5;
        liftInitial = lift.getCurrentPosition()/ticksPerRotation;
        ticksPerRotation = lift.getMotorType().getTicksPerRev();
        liftPos = (lift.getCurrentPosition()/ticksPerRotation)-liftInitial;
        boolean flipAllowed = true;
        double intakePower = 0;
        double newPower = 0;

        telemetry.addData("This program starts at half power. Driver 1, please note", "LT is full power, LB is 3/4 power, RB is half power, RT is 1/4 power");
        telemetry.update();
        //initial = lift.getCurrentPosition()/ticksPerRotation;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentVoltage = getBatteryVoltage();
            telemetry.addData("Voltage", Double.toString(currentVoltage));
            telemetry.addData("liftPos", liftPos);
            telemetry.update();
            //telemetry.addData("Controls 1: ", "LT is full power, LB is 3/4 power, RB is half power, RT is 1/4 power");
            //telemetry.addData("Controls 1 Cont", "a is 0.6 and b is 1 for blue side ducks, x is 0.6 and y is 1 for red side ducks. use the lower powers");
            //telemetry.addData("Controls 2", "LT is open (hold it for a few seconds), claw should auto-close, RT is manual close (if needed)");
            //telemetry.addData("Controls2", "a = ground, x = 1, y = 2, b = 3, and left joystick is manual lift adjustment");
            liftPos = (lift.getCurrentPosition()/ticksPerRotation)-liftInitial;
            //telemetry.addData("liftPos", Double.toString(position));
            //telemetry.addData("gamepad2.left_stick_y", Double.toString(gamepad2.left_stick_y));
            //telemetry.update();
            //all shutoff commands start as false, startup start as true
            intakePower = intake.getPower();
            if (gamepad1.a) {
                newPower = 0;
            }
            else if (gamepad1.x) {
                newPower = -0.5;
            }
            else if (gamepad1.y) {
                newPower = -0.75;
            }
            else if (gamepad1.b) {
                newPower = -1;
            }
            intake.setPower(newPower);
            telemetry.addData(Double.toString(newPower), " new power");
            telemetry.update();

            //Mecanum
            drivePower = 1;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            if (Math.abs(rx) <= 0.05) {
                rx = 0;
            }
            /*if (gamepad1.right_trigger > 0.05) {
                bucket.setPosition(bucketClosePos);
            }
            if (gamepad1.left_trigger > 0.05) {
                bucket.setPosition(bucketOpenPos);
            }
             */
            if (gamepad2.dpad_up) {
                arm.setPosition(.9);
            }
            else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                arm.setPosition(.7);
            }
            else if (gamepad2.dpad_down) {
                arm.setPosition(.48);
            }
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            //drivePower is the fraction of the power used
            double frontLeftPower = (drivePower*(y + x - rx)) / denominator;
            double backLeftPower = (drivePower*(y - x - rx)) / denominator;
            double frontRightPower = (drivePower*(y - x + rx)) / denominator;
            double backRightPower = (drivePower*(y + x + rx)) / denominator;
            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);
            if (gamepad2.left_stick_y > 0.1){
                liftPower = (0.05*gamepad2.left_stick_y);
            }
            else if (gamepad2.left_stick_y < -0.1){
                liftPower = (0.5*gamepad2.left_stick_y);
            }
            else{
                liftPower = (0.05);
                //position = 2;
            }
            lift.setPower(liftPower);
            /*
            if (gamepad2.a == true){
                arm.setPosition(0.35);
            }
            else if (gamepad2.b == true){
                arm.setPosition(0.875);
            }
            if (gamepad2.x == true){
                claw.setPosition(0.5);
            }
            else if (gamepad2.y == true){
                telemetry.addData("", "");
                telemetry.update();
                claw.setPosition(1);
            }
            */

        }
    }
    double getBatteryVoltage(){
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor){
            double voltage = sensor.getVoltage();
            if (voltage > 0){
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}