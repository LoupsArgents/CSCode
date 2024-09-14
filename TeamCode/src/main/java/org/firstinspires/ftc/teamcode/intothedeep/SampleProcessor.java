package org.firstinspires.ftc.teamcode.intothedeep;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class SampleProcessor extends LinearOpMode implements VisionProcessor {
    enum Color {
        RED,
        BLUE,
        YELLOW;
    }
    MatOfPoint largestContour;
    int vertices = 0;
    Color color = Color.YELLOW;
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
            telemetry.addData("Vertices", sp.getNumVertices());
            telemetry.update();
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
        Scalar blueLowHSV = new Scalar(90, 75, 100); //lowS used to be 120
        Scalar blueHighHSV = new Scalar(130, 255, 255);
        Scalar yellowLowHSV = new Scalar(14, 100, 80);
        Scalar yellowHighHSV = new Scalar(28, 255, 255);
        Mat mat = new Mat();
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        if(mat.empty()) return frame;
        Mat original = frame;
        //Trying blue first
        Mat thresh = new Mat();
        Scalar lowHSV;
        Scalar highHSV;
        if(color == Color.BLUE){
            lowHSV = blueLowHSV;
            highHSV = blueHighHSV;
        }else if(color == Color.RED){
            lowHSV = redFirstLowHSV;
            highHSV = redFirstHighHSV;
        }else{
            lowHSV = yellowLowHSV;
            highHSV = yellowHighHSV;
        }
        if(color != Color.RED) {
            Core.inRange(mat, lowHSV, highHSV, thresh);
        }else{
            Core.inRange(mat, lowHSV, highHSV, thresh);
            Mat newThing = new Mat();
            Core.inRange(mat, redSecondLowHSV, redSecondHighHSV, newThing);
            Core.bitwise_or(newThing, thresh, thresh);
        }
        Mat edges = new Mat();
        Mat testMat = new Mat();
        Core.bitwise_and(original, original, testMat, thresh);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat m = new Mat();
        Core.bitwise_and(original, original, m, thresh);
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.Canny(m, edges, 50, 250);
        //edges.copyTo(testMat);
        double maxArea = 0;
        for(MatOfPoint p : contours) {
            if(Imgproc.contourArea(p) < 100){
                continue;
            }
            if(Imgproc.contourArea(p) > maxArea){
                maxArea = Imgproc.contourArea(p);
                largestContour = p;
            }
        }
        /*if(largestContour != null) {
            MatOfPoint2f input = new MatOfPoint2f(largestContour.toArray());
            RotatedRect rectangle = Imgproc.minAreaRect(input);
            Point[] points = new Point[4];
            rectangle.points(points);
            //draw rotated rectangle
            for (int i = 0; i < 4; i++) {
                Imgproc.line(testMat, points[i], points[(i + 1) % 4], new Scalar(70, 200, 200), 5);
            }
        }*/
        if(largestContour != null){
            MatOfPoint2f mp2f = new MatOfPoint2f();
            largestContour.convertTo(mp2f, CvType.CV_32F);
            MatOfPoint2f approximation = new MatOfPoint2f();
            Imgproc.approxPolyDP(mp2f, approximation, .015*Imgproc.arcLength(mp2f, true), true);
            MatOfPoint approxMOP = new MatOfPoint();
            approximation.convertTo(approxMOP, CvType.CV_32S);
            ArrayList<MatOfPoint> temp = new ArrayList<>();
            temp.add(approxMOP);
            Point[] arr = approxMOP.toArray();
            vertices = arr.length;
            Imgproc.drawContours(testMat, temp, -1, new Scalar(70, 200, 200), 5);
        }
        testMat.copyTo(frame);
        return frame;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public int getNumVertices(){return vertices;}
    public void setColor(Color c){color = c;}
    public Color getColor(){return color;}
}
