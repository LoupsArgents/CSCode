package org.firstinspires.ftc.teamcode.intothedeep;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class SampleProcessor extends LinearOpMode implements VisionProcessor {
    @Override
    public void runOpMode() throws InterruptedException {
        CameraName webcam = hardwareMap.get(CameraName.class, "Webcam 1");
        SampleProcessor sp = new SampleProcessor();
        VisionPortal portal = VisionPortal.easyCreateWithDefaults(webcam, sp);
        portal.resumeStreaming();
        while(portal.getCameraState() != VisionPortal.CameraState.STREAMING){
            telemetry.addData("Status", "not ready");
            telemetry.update();
        }
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){

        }
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Scalar redFirstLowHSV = new Scalar(0, 70, 70);
        Scalar redFirstHighHSV = new Scalar(10, 255, 255);
        Scalar redSecondLowHSV = new Scalar(160, 70, 70);
        Scalar redSecondHighHSV = new Scalar(180, 255, 255);
        Scalar blueLowHSV = new Scalar(100, 130, 100);
        Scalar blueHighHSV = new Scalar(120, 255, 255);
        Scalar yellowLowHSV = new Scalar(14, 100, 80);
        Scalar yellowHighHSV = new Scalar(28, 255, 255);
        Mat mat = new Mat();
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        if(mat.empty()) return frame;
        Mat original = frame;
        //Trying blue first
        Mat blueThresh = new Mat();
        Core.inRange(mat, blueLowHSV, blueHighHSV, blueThresh);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blueThresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Mat testMat = new Mat();
        blueThresh.copyTo(testMat);
        Imgproc.drawContours(testMat, contours, -1, new Scalar(90, 255, 150), 5);
        testMat.copyTo(frame);
        return frame;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
