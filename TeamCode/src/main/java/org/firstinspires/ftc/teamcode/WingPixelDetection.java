package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

import android.graphics.Canvas;

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

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@TeleOp
public class WingPixelDetection extends OldPPBotBasicDF {
    public void runOpMode(){
        VisionPortal portal;
        initializeHardware();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        WingPixelProcessor p = new WingPixelProcessor();
        portal = VisionPortal.easyCreateWithDefaults(webcam, p);
        portal.resumeStreaming();
        waitForStart();
        centerOnClosestStack(p);
        while(opModeIsActive()){

        }
    }
    public void centerOnClosestStack(WingPixelProcessor processor){ //current - diagonal movement
        double power = .35;
        Point pixelPos = processor.getClosestPixelPos();
        //^^^ this is the input of CV on this algorithm - telling us whether we're left/right of center and how much
        //which is why (see getClosestPixelPos() method below) it's important that these
        //coordinates include the left/right center of the detected object
        double multiplier;
        while(opModeIsActive() && (/*Math.abs(pixelPos.x-320) > 10 ||*/ pixelPos.y < 300)) {
            RobotLog.aa("DistanceFromCenter", String.valueOf(Math.abs(pixelPos.x - 320)));
            motorBL.setPower(power * multiplierBL);
            motorBR.setPower(power * multiplierBR);
            motorFL.setPower(power * multiplierFL);
            motorFR.setPower(power * multiplierFR);
            double proportionalConstant = -.01; // used to be -.5, then -.3, then -.03; Desmos said -0.00344828
            pixelPos = processor.getClosestPixelPos();
            if(Math.abs(pixelPos.x - 320) < 10){
                motorBL.setPower(power * multiplierBL);
                motorBR.setPower(power * multiplierBR);
                motorFL.setPower(power * multiplierFL);
                motorFR.setPower(power * multiplierFR);
                continue;
            }else if(pixelPos.x < 320){ //go left (reduce FR, BL)
                multiplier = 1 + (Math.abs(320 - pixelPos.x) * proportionalConstant);
                telemetry.addData("multiplier", Double.toString(multiplier));
                telemetry.update();
                RobotLog.aa("Going", "left");
                motorFR.setPower(power * multiplierFR); //all used to be negative for pole radar, positive for cone radar
                motorBL.setPower(power * multiplierBL); //multipliers were on fr and bl for pole radar
                motorFL.setPower(power * multiplierFL * multiplier);
                motorBR.setPower(power * multiplierBR * multiplier);
            }else if(pixelPos.x > 320){ //go right (reduce FL, BR)
                multiplier = 1 + (Math.abs(320 - pixelPos.x) * proportionalConstant);
                telemetry.addData("multiplier", Double.toString(multiplier));
                telemetry.update();
                RobotLog.aa("Going", "right");
                motorFL.setPower(power * multiplierFL);
                motorBR.setPower(power * multiplierBR);
                motorFR.setPower(power * multiplierFR * multiplier);
                motorBL.setPower(power * multiplierBL * multiplier);
            }
        }
        stopMotors();
    }
    public void strafeCenterOnClosestStack(WingPixelDetection.WingPixelProcessor processor){ //an earlier stepping-stone version
        double power = .27;
        double initialHeading = newGetHeading();
        double currentHeading = initialHeading;
        //this is a left/right strafe based on the position of the pixel
        //and using the gyro to keep heading consistent
        //closest precedent is phase 1/3 of old cone radar
        Point pixelPos = processor.getClosestPixelPos();
        double multiplier = 1;
        while(Math.abs(pixelPos.x - 320) > 10 && opModeIsActive()){
            //pixelPos.x is a 0-640 scale
            //if > 320, it's to the right and we need to go right
            //if < 320, it's to the left and we need to go left
            //option: the sort of continuous sensor input we do in cone radar
            //not sure how this deals with motion blur - it may not work or we may need
            //to go very slow.
            //other option: do math based on the y-value, camera angle, etc. (trig?) to figure out
            //how far right/left we think we need to go, execute that, and then maybe (if needed)
            //do the same thing again.
            currentHeading = newGetHeading();
            pixelPos = processor.getClosestPixelPos();

            if(pixelPos.x > 320){ //go right
                if(currentHeading - initialHeading >= 0){
                    multiplier = .1*(currentHeading-initialHeading)+1;
                    motorFL.setPower(power*multiplier*multiplierFL);
                    motorBL.setPower(-power*multiplierBL);
                    motorFR.setPower(-power*multiplier*multiplierFR);
                    motorBR.setPower(power*multiplierBR);
                }else{
                    multiplier = -.1*(currentHeading-initialHeading)+1;
                    motorFR.setPower(-power*multiplierFR);
                    motorBR.setPower(power*multiplier*multiplierBR);
                    motorFL.setPower(power*multiplierFL);
                    motorBL.setPower(-power*multiplier*multiplierBL);
                }
            }else{ //go left
                if(currentHeading > initialHeading){
                    multiplier = .1*(currentHeading-initialHeading)+1;
                    motorFL.setPower(-power*multiplierFL);
                    motorBL.setPower(power*multiplier*multiplierBL);
                    motorFR.setPower(power*multiplierFR);
                    motorBR.setPower(-power*multiplier*multiplierBR);
                }else{
                    multiplier = -.1*(currentHeading-initialHeading)+1;
                    motorFR.setPower(power*multiplier*multiplierFR);
                    motorBR.setPower(-power*multiplierBR);
                    motorFL.setPower(-power*multiplier*multiplierFL);
                    motorBL.setPower(power*multiplierBL);
                }
            }
        }
        stopMotors();

    }
     class WingPixelProcessor implements VisionProcessor {
        int minPixelBoxArea = 1500;
        Point closestPixelPos = new Point(400, 400);
        Rect closestPixelRect;
        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos){
            Mat mat = new Mat();
            Mat original = frame;
            Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                return frame;
            }
            Scalar lowPurpleHSV = new Scalar(117, 40, 20); //purple pixels
            Scalar highPurpleHSV = new Scalar(150, 255, 255); //120-150 should do for hue for starters
            Scalar lowYellowHSV = new Scalar(14, 40, 80); //before, last was 80 (excluded brown but really that's not necessary) then 20, second-to-last was 100 then 80
            Scalar highYellowHSV = new Scalar(28, 255, 255);
            Scalar lowGreenHSV = new Scalar(45,100,25); //the green filter is somehow perfect and has no problems. I wish all the other ones were like that.
            Scalar highGreenHSV = new Scalar(75, 255, 255);
            Scalar lowWhiteHSV = new Scalar(0,0,180); //last was 150
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
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            Random rng = new Random(12345);
            for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
                Imgproc.drawContours(masked, contoursPolyList, i, color);
                if(boundRect[i].area() > minPixelBoxArea) {
                    //draw all bounding rectangles that pass the minimum notability threshold onto the image
                    Imgproc.rectangle(masked, boundRect[i].tl(), boundRect[i].br(), color, 2);
                    telemetry.addData("Rect", boundRect[i]);

                }
                //Imgproc.circle(masked, centers[i], (int) radius[i][0], color, 2);
            }
            //give me the bounding box with highest y???
            Rect maxRect = new Rect(0,0,10,10);
            for(Rect r : boundRect){
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
            RobotLog.aa("Box", maxRect.toString());
            closestPixelPos = new Point(maxRect.x, maxRect.y); //this is what actually informs our algorithm - see function below for a bit more processing
            closestPixelRect = maxRect;
            telemetry.addData("BoundingBox", maxRect);
            telemetry.update();
            masked.copyTo(frame);
            return frame;
        }

         @Override
         public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            //we just have to override this so it doesn't get mad
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
     }
}
