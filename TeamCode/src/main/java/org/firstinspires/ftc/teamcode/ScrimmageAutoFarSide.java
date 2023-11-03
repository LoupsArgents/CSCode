package org.firstinspires.ftc.teamcode;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
@Autonomous
public class ScrimmageAutoFarSide extends PPBotCSDF {
    public void runOpMode(){
        initializeHardware();
        processor.setAlliance(1);
        closeClaw();
        sleep(500);
        arm.setTargetPosition(armSpikeMarkPos);
        turret.setPosition(turretPos);
        arm.setPower(0.5);
        String result = "Center";
        while(opModeInInit()){
            result = processor.getResult();
            telemetry.addData("Result", result);
            telemetry.update();
        }
        waitForStart();
        goStraight(.5, 16);
        if(result.equals("Left")){
            absoluteHeading(.3, 45);
            goStraight(.5, 2);
            openClaw();
        }else if(result.equals("Center")){

        }else if(result.equals("Right")){

        }
        while(opModeIsActive()){

        }
    }
}
