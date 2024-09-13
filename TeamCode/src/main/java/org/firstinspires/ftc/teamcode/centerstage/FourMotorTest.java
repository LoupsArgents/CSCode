/*
Copyright 2023 FIRST Tech Challenge Team 20153

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
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

public class FourMotorTest extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotorEx motorFR;
    private DcMotorEx motorFL;
    private DcMotorEx motorBR;
    private DcMotorEx motorBL;
    private int FLticks;
    private int FRticks;
    private int BRticks;
    private int BLticks;
    private int FLstart;
    private int FRstart;
    private int BRstart;
    private int BLstart;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean hasTimerRun = false;
    private double timeElapsed;
    private double power;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        power = 1;
        motorFR = hardwareMap.get(DcMotorEx.class, "motor3");
        motorFL = hardwareMap.get(DcMotorEx.class, "motor2"); // a bit faster
        motorBR = hardwareMap.get(DcMotorEx.class, "motor1");
        motorBL = hardwareMap.get(DcMotorEx.class, "motor0"); // a bit faster
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRstart = motorFR.getCurrentPosition();
        FLstart = motorFL.getCurrentPosition();
        BRstart = motorBR.getCurrentPosition();
        BLstart = motorBL.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("dpad", "up is front left, right is front right, down is back right, and left is back left");
            telemetry.addData("letters", "y is front left, b is front right, a is back right, and x is back left");
            /*telemetry.addData("FR", motorFR.getVelocity());
            telemetry.addData("BR", motorBR.getVelocity());
            telemetry.addData("FL", motorFL.getVelocity());
            telemetry.addData("BL", motorBL.getVelocity());*/
            telemetry.addData("FR", (FRticks-FRstart)/timeElapsed); // ticks/time elapsed
            telemetry.addData("BR", (BRticks-BRstart)/timeElapsed);
            telemetry.addData("FL", (FLticks-FLstart)/timeElapsed);
            telemetry.addData("BL", (BLticks-BLstart)/timeElapsed);
            telemetry.addData("FR ticks", FRticks-FRstart);
            telemetry.addData("FL ticks", -1*(FLticks-FLstart));
            telemetry.addData("BR ticks", BRticks-BRstart);
            telemetry.addData("BL ticks", -1*(BLticks-BLstart));
            telemetry.addData("time elapsed", timeElapsed);
            telemetry.update();
            if (gamepad1.dpad_up){
                motorFL.setPower(2620/2660);
            }
            else if (gamepad1.dpad_right){
                motorFR.setPower(2620/2720);
            }
            else if (gamepad1.dpad_down){
                motorBR.setPower(2620/2800);
            }
            else if (gamepad1.dpad_left){
                motorBL.setPower(1);
            }
            else if (gamepad1.y){
                motorFL.setPower(0.13);
            }
            else if (gamepad1.b){
                motorFR.setPower(0.13);
            }
            else if (gamepad1.a){
                motorBR.setPower(0.13);
            }
            else if (gamepad1.x){
                motorBL.setPower(0.13);
            } //fl, bl, fr, br
            else if (gamepad1.back){
                if (!hasTimerRun){
                    timer.reset();
                    hasTimerRun = true;
                }
                motorFR.setPower(power);
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorBR.setPower(power);
                FLticks = motorFL.getCurrentPosition();
                FRticks = motorFR.getCurrentPosition();
                BLticks = motorBL.getCurrentPosition();
                BRticks = motorBR.getCurrentPosition();
                timeElapsed = timer.milliseconds();
            }// ticks/time elapsed
            else if (gamepad1.start){
                motorFR.setPower(0.13);
                motorFL.setPower(0.13);
                motorBR.setPower(0.13);
                motorBL.setPower(0.13);
            }
            else{
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
            }



        }
    }
}

