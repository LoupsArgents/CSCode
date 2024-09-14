package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import java.util.Iterator;
import java.util.List;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ServoImplEx; // for Axon
import com.qualcomm.robotcore.hardware.PwmControl; // for Axon


@TeleOp
@Disabled
public class FreightFrenzyInfoSession extends LinearOpMode {
    DcMotorEx motorFR;
    DcMotorEx motorFL;
    DcMotorEx motorBR;
    DcMotorEx motorBL;
    DcMotorEx lift;
    Servo claw;
    DistanceSensor clawDist;

    double ticksPerRotation;
    double liftInitial;
    double liftPos;
    boolean clawClosed = false;
    double clawClosePos = 0.875; //was 1.0, then 0.9
    double clawOpenPos = 0.75;
    double liftIdealPos = 0.0;
    boolean isJoysticking = false;
    boolean liftHappyPlace = true;
    double clawDistReading = 100;
    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");
        clawDist = hardwareMap.get(DistanceSensor.class, "clawDist");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        ticksPerRotation = lift.getMotorType().getTicksPerRev();
        liftInitial = lift.getCurrentPosition()/ticksPerRotation;
        liftPos = (lift.getCurrentPosition()/ticksPerRotation)-liftInitial;

        claw.setPosition(clawOpenPos);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("liftPos", liftPos);
            telemetry.addData("clawDistReading", clawDistReading);
            telemetry.addData("gamepad1.left_trigger", gamepad1.left_trigger);
            telemetry.addData("gamepad1.right_trigger", gamepad1.right_trigger);
            telemetry.addData("test", "test");
            telemetry.update();

            liftPos = (lift.getCurrentPosition()/ticksPerRotation)-liftInitial;
            clawDistReading = clawDist.getDistance(DistanceUnit.INCH);

            //mecanum
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double multiplier = 0.7;

            motorFL.setPower(frontLeftPower * multiplier);
            motorBL.setPower(backLeftPower * multiplier);
            motorFR.setPower(frontRightPower * multiplier);
            motorBR.setPower(backRightPower * multiplier);

            //lift controls
            double liftStallPower = 0.05 * liftPos;
            double liftPower = -gamepad2.left_stick_y;
            if (liftPower < 0) { //we're trying to go down
                liftPower *= 0.5;
            } else {
                liftPower *= 0.75;
            }
            double liftError = liftIdealPos - liftPos;
            double liftTolerance = 0.01;
            double Kp = 16; //was 30, then 15 (too high), then 12.5, 7.5 was too low
            if (Math.abs(liftError) < liftTolerance) {
                liftHappyPlace = true;
            } else {
                liftHappyPlace = false;
            }
            if (!isJoysticking && !liftHappyPlace) {
                lift.setPower(liftError*Kp + liftStallPower);
            } else if (!isJoysticking) { //but liftHappyPlace is true
                liftStallPower = 0.05 * liftPos; //m was 0.41
                if (liftStallPower > 0.2) {
                    liftStallPower = 0.2;
                }
                lift.setPower(liftStallPower);
            }
            if ((liftPower > 0.05 && liftPos < 0.75) || (liftPower < -0.05 && liftPos > 0)) {
                isJoysticking = true;
                liftHappyPlace = true;
                liftIdealPos = liftPos;
                lift.setPower(liftPower);
            } else {
                isJoysticking = false;
            }


            //claw
            if (gamepad1.left_trigger > 0.05) {
                telemetry.addData("opening", "");
                claw.setPosition(clawOpenPos);
                clawClosed = false;
            } else if (((clawDistReading < 3 && clawClosed == false) || gamepad1.right_trigger > 0.05)) {
                telemetry.addData("closing", "");
                if (!clawClosed) {
                    claw.setPosition(clawClosePos);
                    clawClosed = true;
                }
            }
        }
    }
}
