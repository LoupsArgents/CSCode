package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
public class OldPPBotBasicDF extends LinearOpMode {
    Blinker control_Hub;
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    double multiplierBL = 0.95; //0.93686; //(3443.0000000/3780.0000000);
    double multiplierBR = 0.95;   //1;// 1;
    double multiplierFR = 1; //1;// 3443.0000000/3677.0000000;
    double multiplierFL = 0.956250; //0.956250; // 3443.0000000/3822.0000000;

    IMU imu;
    double previousHeading;
    double processedHeading;
    public void runOpMode(){
        initializeHardware();
        waitForStart();
    }
    public void initializeHardware(){
            control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
            motorFR = hardwareMap.get(DcMotor.class, "motor0");
            motorFL = hardwareMap.get(DcMotor.class, "motor1");
            motorBR = hardwareMap.get(DcMotor.class, "motor2");
            motorBL = hardwareMap.get(DcMotor.class, "motor3");
            imu = hardwareMap.get(IMU.class, "imu");
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
    public double newGetHeading(){
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double headingChange = currentHeading - previousHeading;
        if(headingChange < -180){
            headingChange += 360;
        }else if(headingChange > 180){
            headingChange -= 360;
        }
        processedHeading += headingChange;
        previousHeading = currentHeading;
        return processedHeading;
    }
        public void stopMotors(){
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
        }
    }
