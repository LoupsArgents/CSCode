package org.firstinspires.ftc.teamcode;

import static org.opencv.core.Core.flip;

import android.graphics.Canvas;

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
@TeleOp
public class TeamPropRecognition extends LinearOpMode {
    public void runOpMode(){
        VisionPortal portal;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //TfodProcessor myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
        TeamPropProcessor teamPropProcessor = new TeamPropProcessor();
        teamPropProcessor.setAlliance(1);
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
        int alliance; //1 is red, -1 is blue
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
            Core.flip(mat, mat, -1);
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
        public void setAlliance(int a){ //0 is not-black HSV filter just for fun use on the board
            alliance = a;
        }
    }
}
