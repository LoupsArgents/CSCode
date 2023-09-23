package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;

import android.graphics.Canvas;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.imgproc.Imgproc;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;
import java.util.Locale;
@TeleOp
public class PixelRecognition extends LinearOpMode {
    public void runOpMode(){
        VisionPortal portal;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //TfodProcessor myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
        TeamPropProcessor teamPropProcessor = new TeamPropProcessor();
        portal = VisionPortal.easyCreateWithDefaults(webcam, teamPropProcessor);
        portal.resumeStreaming();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Result", teamPropProcessor.getResult());
            telemetry.update();
        }
    }
     class TeamPropProcessor implements VisionProcessor {
        double leftVal;
        double rightVal;
        double centerVal;
        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Mat mat = new Mat();
            Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                return frame;
            }
            Scalar lowHSV = new Scalar(160, 70, 70);
            Scalar highHSV = new Scalar(180, 255, 255);
            Mat thresh = new Mat();
            // Get a black and white image of red objects
            Core.inRange(mat, lowHSV, highHSV, thresh);

            mat.release();
            thresh.copyTo(frame);
            Imgproc.line(frame, new Point(180, 0), new Point(180, 500), new Scalar(100, 100, 100), 4, Imgproc.LINE_8);
            Imgproc.line(frame, new Point(460, 0), new Point(460, 500), new Scalar(100, 100, 100), 4, Imgproc.LINE_8);
            Mat left = thresh.submat(0, thresh.rows(), 0, thresh.cols()/3);
            Mat center = thresh.submat(0, thresh.rows(), thresh.cols()/3, 2*thresh.cols()/3);
            Mat right = thresh.submat(0, thresh.rows(), 2 * thresh.cols() / 3, thresh.cols());
            //ok now we have left, right, center: do pixel avgs.
            Core.extractChannel(left, left, 0);
            Core.extractChannel(right, right, 0);
            Core.extractChannel(center, center, 0);
            leftVal = Core.mean(left).val[0];
            rightVal = Core.mean(right).val[0];
            centerVal = Core.mean(center).val[0];
           // telemetry.addData("Left", leftVal);
            //telemetry.addData("Center", centerVal);
            //telemetry.addData("Right", rightVal);
            telemetry.update();
            return left;
            //frame.release();
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }
        public String getResult(){
            if(leftVal > rightVal && leftVal > centerVal){
                return "Left";
            }else if(rightVal > leftVal && rightVal > centerVal){
                return "Right";
            }else{
                return "Center";
            }
        }
    }
}
