package org.firstinspires.ftc.teamcode;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;
import static org.opencv.imgproc.Imgproc.putText;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Imgproc.*;

import java.util.*;

@TeleOp
public class WingPixelDetection extends LinearOpMode{
    public void runOpMode(){
        VisionPortal portal;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        WingPixelDetection.WingPixelProcessor p = new WingPixelDetection.WingPixelProcessor();
        portal = VisionPortal.easyCreateWithDefaults(webcam, p);
        portal.resumeStreaming();
        waitForStart();
        while(opModeIsActive()){

        }
    }
    public void centerOnClosestStack(){

    }
     class WingPixelProcessor implements VisionProcessor {
        int minPixelBoxArea = 1500;
        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos){
            Mat mat = new Mat();
            Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                return frame;
            }
            Scalar lowPurpleHSV = new Scalar(117, 40, 20); //purple pixels
            Scalar highPurpleHSV = new Scalar(150, 255, 255); //120-150 should do for hue for starters
            Scalar lowYellowHSV = new Scalar(15, 100, 80);
            Scalar highYellowHSV = new Scalar(27, 255, 255);
            Scalar lowGreenHSV = new Scalar(45,100,50);
            Scalar highGreenHSV = new Scalar(70, 255, 255);
            Scalar lowWhiteHSV = new Scalar(0,0,180); //last was 150
            Scalar highWhiteHSV = new Scalar(180, 20, 255);
            Mat purpleThresh = new Mat();
            Mat yellowThresh = new Mat();
            Mat greenThresh = new Mat();
            Mat whiteThresh = new Mat();
            // Get a black and white image of (color) objects
            Core.inRange(mat, lowPurpleHSV, highPurpleHSV, purpleThresh);
            Core.inRange(mat, lowYellowHSV, highYellowHSV, yellowThresh);
            Core.inRange(mat, lowGreenHSV, highGreenHSV, greenThresh);
            Core.inRange(mat, lowWhiteHSV, highWhiteHSV, whiteThresh);
            Mat testOutput = new Mat();
            Core.bitwise_or(purpleThresh, yellowThresh, testOutput);
            Core.bitwise_or(testOutput, greenThresh, testOutput);
            Core.bitwise_or(testOutput, whiteThresh, testOutput);
            Mat masked = new Mat();
            //color the white portion of thresh in with color
            //output into masked
            Mat thing = new Mat(480, 640, CvType.CV_8UC3, new Scalar(140, 70, 200)); //purple was 140 70 200
            Core.bitwise_and(thing, thing, masked, testOutput);
            //Scalar average = Core.mean(masked, thresh);
            Mat scaledMask = new Mat();
            //scale the average saturation to 150
            //masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);
            testOutput.copyTo(frame);
            Mat cannyOutput = new Mat();
            int threshold = 170;
            Imgproc.blur(masked, masked, new Size(5, 5));
            Imgproc.Canny(masked, cannyOutput, threshold, threshold * 2);
            Imgproc.blur(cannyOutput, cannyOutput, new Size(5, 5));
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            Point[] centers = new Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }
            Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            Random rng = new Random(12345);
            for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
                Imgproc.drawContours(masked, contoursPolyList, i, color);
                if(boundRect[i].area() > minPixelBoxArea) {
                    Imgproc.rectangle(masked, boundRect[i].tl(), boundRect[i].br(), color, 2);
                    telemetry.addData("Rect", boundRect[i]);

                }
                //Imgproc.circle(masked, centers[i], (int) radius[i][0], color, 2);
            }
            //give me the bounding box with highest y???
            Rect maxRect = new Rect(0,0,10,10);
            for(Rect r : boundRect){
                if(r.y > maxRect.y && r.area() > minPixelBoxArea){
                    maxRect = r;
                    Imgproc.putText(masked, Double.toString(r.area()), new Point(r.x, r.y), FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
                }
            }
            Rect r = new Rect(320, 100, 10, 10);
            Mat regionOfInterest = masked.submat(r);
            Scalar s = Core.mean(regionOfInterest);
            telemetry.addData("Color", s);
            Imgproc.rectangle(masked, r, new Scalar(255, 255, 255));
            RobotLog.aa("Box", maxRect.toString());
            telemetry.addData("BoundingBox", maxRect);
            telemetry.update();
            masked.copyTo(frame);
            return frame;
        }

         @Override
         public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

         }
     }
}
