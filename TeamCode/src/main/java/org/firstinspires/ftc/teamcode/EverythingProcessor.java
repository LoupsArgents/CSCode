package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
@Disabled
public class EverythingProcessor extends LinearOpMode implements VisionProcessor {
    String updates = "";
    double leftVal;
    double rightVal;
    double centerVal;
    int alliance; //1 is red, -1 is blue
    int setting; //0 is team prop, 1 is wing pixel
    int minPixelBoxArea = 2500; //up from 1500 so it doesn't see parts of the robot as pixels
    Point closestPixelPos = new Point(400, 400);
    Rect closestPixelRect;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        updates = "";
        Core.flip(frame, frame, -1);
        if(setting == 0){
            return doPropProcessing(frame);
        }else{
            return doWingProcessing(frame);
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //gotta override this so it doesn't get mad
    }
    public Object doPropProcessing(Mat frame){
        Mat mat = new Mat();
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return frame;
        }
        Scalar lowHSV;
        Scalar highHSV;
        if(alliance == 1) {
            lowHSV = new Scalar(160, 70, 70);
            highHSV = new Scalar(180, 255, 255);
        }else{
            lowHSV = new Scalar(100, 70, 70);
            highHSV = new Scalar(120, 255, 255);
        }
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
        //so left, right, and center val are what we need
       //  telemetry.addData("Left", leftVal);
       // telemetry.addData("Center", centerVal);
       // telemetry.addData("Right", rightVal);
        updates += ("Left: " + leftVal + "\nCenter: " + centerVal + "\nRight: " + rightVal);
       // telemetry.update();
        return left;
        //frame.release();
    }
    public Point getClosestPixelPos(){
        //what we're doing here is actually getting the center of the top edge of the bounding box
        //y is only important relatively, so making it the top edge doesn't hurt anything as long as we're consistent (which we are)
        //x, however (as you can see in centerOnClosestStack above) needs to represent the center of the pixel
        //
        return new Point(closestPixelPos.x + (closestPixelRect.width/2), closestPixelPos.y);
    }
    public Rect getClosestPixelRect(){
        return this.closestPixelRect;
    }
    public Object doWingProcessing(Mat frame){
        Mat mat = new Mat();
        Mat original = frame;
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return frame;
        }
        Scalar lowPurpleHSV = new Scalar(117, 40, 20); //purple pixels
        Scalar highPurpleHSV = new Scalar(150, 255, 255); //120-150 should do for hue for starters
        Scalar lowYellowHSV = new Scalar(14, 100, 80); //changed lower brightness threshold b/c it saw the field as yellow lol
        Scalar highYellowHSV = new Scalar(28, 255, 255);
        Scalar lowGreenHSV = new Scalar(45,100,25);
        //the green filter is somehow perfect and has no problems. I wish all the other ones were like that.
        Scalar highGreenHSV = new Scalar(75, 255, 255);
        Scalar lowWhiteHSV = new Scalar(0,0,200); //last was 180 - updated 11/4/23 for PP offseason bot different camera angle
        Scalar highWhiteHSV = new Scalar(180, 20, 255);
        Mat purpleThresh = new Mat();
        Mat yellowThresh = new Mat();
        Mat greenThresh = new Mat();
        Mat whiteThresh = new Mat();
        // Get a black and white image of objects that are purple, yellow, green, or white
        Core.inRange(mat, lowPurpleHSV, highPurpleHSV, purpleThresh);
        Core.inRange(mat, lowYellowHSV, highYellowHSV, yellowThresh);
        Core.inRange(mat, lowGreenHSV, highGreenHSV, greenThresh);
        Core.inRange(mat, lowWhiteHSV, highWhiteHSV, whiteThresh);
        Mat testOutput = new Mat();
        Core.bitwise_or(purpleThresh, yellowThresh, testOutput);
        Core.bitwise_or(testOutput, greenThresh, testOutput);
        Core.bitwise_or(testOutput, whiteThresh, testOutput); //combine the black and white images into one black and white image of things that are game elements
        //well, it also includes things that are close in color to game elements, but that's not an issue.
        Mat masked = new Mat();
        //color the white portion of thresh in with color from original image
        //output into masked
        //Mat thing = new Mat(480, 640, CvType.CV_8UC3, new Scalar(140, 70, 200)); //purple was 140 70 200
        Core.bitwise_and(original, original, masked, testOutput);
        //Scalar average = Core.mean(masked, thresh);
        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        //masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);
        testOutput.copyTo(frame);
        Mat cannyOutput = new Mat();
        int threshold = 170;
        Imgproc.blur(masked, masked, new Size(6, 6)); //was 5, 5
        Imgproc.Canny(masked, cannyOutput, threshold, threshold * 2); //edge detection wizardry copied from OpenCV tutorials - works great.
        Imgproc.blur(cannyOutput, cannyOutput, new Size(5, 5));
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); //contour wizardry copied from OpenCV tutorials
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            //bounding box wizardry copied from OpenCV tutorials
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }
        Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);
        //List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        //for (MatOfPoint2f poly : contoursPoly) {
        //    contoursPolyList.add(new MatOfPoint(poly.toArray()));
        //}
        Random rng = new Random(12345);
            /*for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
                Imgproc.drawContours(masked, contoursPolyList, i, color);
                if(boundRect[i].area() > minPixelBoxArea) {
                    //draw all bounding rectangles that pass the minimum notability threshold onto the image
                    Imgproc.rectangle(masked, boundRect[i].tl(), boundRect[i].br(), color, 2);
                    telemetry.addData("Rect", boundRect[i]);

                }
                //Imgproc.circle(masked, centers[i], (int) radius[i][0], color, 2);
            }*/
        //give me the bounding box with highest y???
        Rect maxRect = new Rect(0,0,10,10);
        for(Rect r : boundRect){
            Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
            if(r.area() > minPixelBoxArea){
                Imgproc.rectangle(masked, r.tl(), r.br(), color, 2);
                updates += "Rect " + r + "\n";
               // telemetry.addData("Rect", r);
            }
            if(r.y > maxRect.y && r.area() > minPixelBoxArea){
                //find the closest notable bounding box
                maxRect = r;
                Imgproc.putText(masked, Double.toString(r.area()), new Point(r.x, r.y), FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
            }
        }
        //Rect r = new Rect(320, 100, 10, 10);
        //Mat regionOfInterest = masked.submat(r);
        //Scalar s = Core.mean(regionOfInterest);
        //telemetry.addData("Color", s);
        //Imgproc.rectangle(masked, r, new Scalar(255, 255, 255));
        //^earlier code for finding average color over a small box to calibrate filters
        //RobotLog.aa("Box", maxRect.toString());
        closestPixelPos = new Point(maxRect.x, maxRect.y); //this is what actually informs our algorithm - see function below for a bit more processing
        closestPixelRect = maxRect;
        updates += "BoundingBox: " + maxRect;
       // telemetry.addData("BoundingBox", maxRect);
        //telemetry.update();
        masked.copyTo(frame); //always change back to masked.copyTo(frame) to see bounding boxes, etc.
        //i have a sinking suspicion that it is, in fact, the yellow filter causing these problems
        //IT IS the yellow filter! i'm sorry for ever doubting you, green filter. you're perfect.
        return frame;
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
    public void setAlliance(int a){ //0 is not-black HSV filter just for fun use on the board
        alliance = a;
    }
    public void setMode(int m){
        this.setting = m;
    }
    public String getUpdates(){ return updates; }
    public double[] getVals(){ //return left, center, right
        double[] d = {leftVal, centerVal, rightVal};
        return d;
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}