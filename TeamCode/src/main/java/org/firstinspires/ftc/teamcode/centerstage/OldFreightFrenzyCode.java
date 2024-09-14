package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp
@Disabled

public class OldFreightFrenzyCode extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private Servo claw;
    private DcMotor lift;
    private double drivePower;
    private double duckPower;
    private double clawPos;
    private static double ticksPerRotation;
    static double initial;
    private double position;
    private double idealPosition;
    private Boolean happyPlaceLift;
    DistanceSensor clawDistance;


    //@Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        lift = hardwareMap.get(DcMotor.class, "lift");
        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDist");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = hardwareMap.get(Servo.class, "claw");
        drivePower = 0.5;
        clawPos = 0.75;
        duckPower = 0;
        ticksPerRotation = lift.getMotorType().getTicksPerRev();
        happyPlaceLift = true;
        idealPosition = (lift.getCurrentPosition()/ticksPerRotation)-initial;
        position = (lift.getCurrentPosition()/ticksPerRotation)-initial;

        telemetry.addData("This program starts at half power. Driver 1, please note", "LT is full power, LB is 3/4 power, RB is half power, RT is 1/4 power");
        telemetry.update();
        initial = lift.getCurrentPosition()/ticksPerRotation;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Controls 1: ", "LT is full power, LB is 3/4 power, RB is half power, RT is 1/4 power");
            telemetry.addData("Controls 1 Cont", "a is 0.6 and b is 1 for blue side ducks, x is 0.6 and y is 1 for red side ducks. use the lower powers");
            telemetry.addData("Controls 2", "LT is open (hold it for a few seconds), claw should auto-close, RT is manual close (if needed)");
            telemetry.addData("Controls2", "a = ground, x = 1, y = 2, b = 3, and left joystick is manual lift adjustment");
            //telemetry.addData("Claw position", claw.getPosition());
            //telemetry.addData("ClawPos", Double.toString(clawPos));
            //telemetry.addData(Double.toString(position), Double.toString(idealPosition));
            telemetry.addData("clawDistance", Double.toString(clawDistance.getDistance(DistanceUnit.INCH)));
            position = (lift.getCurrentPosition()/ticksPerRotation)-initial;
            telemetry.addData("liftPos", Double.toString(position));
            telemetry.addData("gamepad2.left_stick_y", Double.toString(gamepad2.left_stick_y));
            telemetry.update();
            //Mecanum
            if (gamepad1.left_trigger > 0.25){ //we don't want this to be hit by accident
                drivePower = 1;
            }
            if (gamepad1.left_bumper){
                drivePower = 0.75;
            }
            if (gamepad1.right_bumper){
                drivePower = 0.5;
            }
            if (gamepad1.right_trigger > 0.25) { // we don't want this hit by accident either
                drivePower = 0.25;
            }
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
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
            //Move lift to ground, level 1, level 2, level 3
            if (Math.abs(position - idealPosition) > 0.02 && opModeIsActive() && happyPlaceLift == false){
                //used to be while
                if (position > idealPosition){
                    lift.setPower(-0.5); //used to be 0.3
                }
                if (position < idealPosition){
                    lift.setPower(0.7); // used to be 0.6, that was slow
                }
                position = (lift.getCurrentPosition()/ticksPerRotation)-initial;
            }
            if(Math.abs(position-idealPosition) < 0.02 && opModeIsActive()){
                lift.setPower(0);
                happyPlaceLift = true;
            }
            if (gamepad2.a){
                idealPosition = 0.005; //USED TO BE 0, then 0.01
                happyPlaceLift = false;
            }
            if (gamepad2.x){ //x 1 y 2 b 3
                idealPosition = 0.18;
                happyPlaceLift = false;
            }
            if (gamepad2.y) {
                idealPosition = 0.4;
                happyPlaceLift = false;
            }
            if (gamepad2.b){
                idealPosition = 0.68;
                happyPlaceLift = false;
            }
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                if ((gamepad2.left_stick_y > 0.05) && position > 0.01) { //this is actually down
                    lift.setPower(-0.5*gamepad2.left_stick_y);
                    position = (lift.getCurrentPosition()/ticksPerRotation)-initial;
                }
                else if (position <= 0.01){
                    lift.setPower(0);
                }
                if ((gamepad2.left_stick_y < -0.05) && position < 0.8){ //this is actually up
                    lift.setPower(-0.5*gamepad2.left_stick_y);
                    position = (lift.getCurrentPosition()/ticksPerRotation)-initial;
                }
                else if (position >= 0.8){
                    lift.setPower(0);
                }
                position = (lift.getCurrentPosition()/ticksPerRotation)-initial;
            }
            else if (happyPlaceLift == true){
                lift.setPower(0);
            }
            position = (lift.getCurrentPosition()/ticksPerRotation)-initial;
            //DuckSpinner
            if (gamepad1.a && opModeIsActive()){
                duckPower = 0.6;
            }
            else if (gamepad1.b && opModeIsActive()){
                duckPower = 1;
            }
            else if (gamepad1.x && opModeIsActive()){
                duckPower = -0.6;
            }
            else if (gamepad1.y && opModeIsActive()){
                duckPower = -1;
            }
            else if (gamepad1.a == false && gamepad1.b == false && gamepad1.x == false && gamepad1.y == false){
                duckPower = 0;
            }

            //SQUISHCLAW
            if(gamepad2.left_trigger > 0.25){
                clawPos = 0.75;
            }
            else if ((clawDistance.getDistance(DistanceUnit.INCH) < 3) && clawPos == 0.75) {
                clawPos = 1;
                gamepad1.rumble(500);
                gamepad2.rumble(1, 1, 500);
            }
            else if (gamepad2.right_trigger > 0.25){
                clawPos = 1;
            }
            claw.setPosition(clawPos);
        }
    }
    //}
}
