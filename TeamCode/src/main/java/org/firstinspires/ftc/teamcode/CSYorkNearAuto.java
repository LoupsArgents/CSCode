package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@Autonomous
public class CSYorkNearAuto extends CSYorkDF {

    public void runOpMode(){
        doRun("Blue");
    }
    public void doRun(String alliance){ //1 is blue, -1 is red
        int allianceNum = 1;
        if(alliance.equals("Blue")) allianceNum = 1;
        if(alliance.equals("Red")) allianceNum = -1;
        String result = onInit(allianceNum);
        //waitForStart();
        onRun(result, allianceNum);
    }
    public String onInit(int alliance){ //returns the team prop result
        initializeHardware();
        processor.setAlliance(alliance);
        processor.setMode(EverythingProcessor.ProcessorMode.PROP);
        ArrayList<Double> rightAverages = new ArrayList<>();
        ArrayList<Double> leftAverages = new ArrayList<>();
        double leftAvg = 0.0;
        double rightAvg = 0.0;
        setInitialPositions();
        while(opModeInInit()){
            double leftCurrent = leftDistance.getDistance(DistanceUnit.INCH);
            if(leftCurrent > 0){
                leftAverages.add(0, leftCurrent);
                if(leftAverages.size() > 5){
                    leftAverages.remove(5);
                }
            }
            if(leftAverages.size() == 5){
                leftAvg = 0.0;
                for(double d : leftAverages){
                    leftAvg += d;
                }
                leftAvg /= 5;
            }
            double rightCurrent = rightDistance.getDistance(DistanceUnit.INCH);
            if(rightCurrent > 0){
                rightAverages.add(0, rightCurrent);
                if(rightAverages.size() > 5){
                    rightAverages.remove(5);
                }
            }
            if(rightAverages.size() == 5){
                rightAvg = 0.0;
                for(double d : rightAverages){
                    rightAvg += d;
                }
                rightAvg /= 5;
            }
        }
        waitForStart();
        String result = getPropResult(leftAvg, rightAvg);
        return result;
    }
    public void setInitialPositions(){
        //wrist slightly up, arm slightly up, flip wrist, arm down
        cameraBar.setPosition(camTuckedIn);
        closeClaw();
        sleep(400);
        wrist.setPosition(wristAlmostDown);
        sleep(250);
        arm1.setPosition(armAlmostDown);
        sleep(500);
        wrist.setPosition(wristTuckedIn);
        sleep(250);
        arm1.setPosition(arm1DownPos);
        sleep(500);
        telemetry.addData("Status", "Ready");
    }
    public void onRun(String result, int alliance){
        activateBackCamera();
        wrist.setPosition(wristAlmostDown);
        doPurplePixel(result, alliance);
        doYellowPixel(result, alliance);
    }
    public void doPurplePixel(String result, int alliance){
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            if(alliance == 1){
                moveForwardLeft(.5, 10.5, 0.0);
            }else if(alliance == -1){
                moveForwardRight(.5, 10.5, 0.0);
            }
            goStraight(.3, 3.5, 0.0);
            openLowerClaw();
            sleep(500);
        }else if(result.equals("Center")){
            //it is roughly 25 inches to the spike mark, so.
            double toSubtract = 0.0;
            if(alliance == 1){
                toSubtract = moveForwardLeft(.55, 4, 0.0);
            }else if(alliance == -1){
                toSubtract = moveForwardRight(.55, 4, 0.0);
            }
            RobotLog.aa("HeadingAfterDiagonal", String.valueOf(newGetHeading()));
            goStraight(.3, 25-toSubtract, 0.0);
            openLowerClaw();
            sleep(500);
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            goStraight(.4, 13, 0.0);
            absoluteHeading(.4, -45.0*alliance);
            absoluteHeading(.2, -45.0*alliance);
            goStraight(.4, 1, -45.0*alliance);
            openLowerClaw();
            sleep(500);
        }
        wrist.setPosition(wristAlmostDown);
        sleep(100);
        arm1.setPosition(armAlmostUp);
    }
    public void doYellowPixel(String result, int alliance) {
       getToBackdrop(result, alliance);
       arm1.setPosition(arm1ScoringPos);
       wrist.setPosition(wristScoringPos);
       absoluteHeading(.2, -90.0*alliance);
       sleep(500);
       positionOnBackdrop(result, alliance);
       sleep(500);
       openUpperClaw();
       sleep(1000);
    }
    public void getToBackdrop(String result, int alliance){
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            goBackward(.3, 3, 0.0 * alliance);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            goBackward(.3, 12, -90.0*alliance);
            if(alliance == 1){
                //strafe left
                strafeLeft(.35, 7, 5, -90.0 * alliance);
            }else if(alliance == -1){
                //strafe right
                strafeRight(.35, 7, 5, -90.0 * alliance);
            }
        }else if(result.equals("Center")){
            goBackward(.3, 5, 0.0);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            goBackward(.3, 8, -90.0*alliance);
            if(alliance == 1){
                moveBackLeft(.35, 8, -90.0*alliance);
            }else{
                moveBackRight(.35, 8, -90.0*alliance);
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            goBackward(.3, 6, -45.0*alliance);
            sleep(250);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            //so we want to have a net move of 5 inches strafe, 20 inches backward
            //goBackward(.3, 20, -90.0 * alliance);
            sleep(250);
            double movedBack = 0.0;
            if(alliance == 1){
                //strafeLeft(.35, 5, 5, -90.0*alliance);
                movedBack = Math.abs(moveBackLeft(.5, 5, -90.0*alliance));
            }else if(alliance == -1){
                //strafeRight(.35, 5, 5, -90.0*alliance);
                movedBack = Math.abs(moveBackRight(.5, 5, -90.0*alliance));
            }
            sleep(100);
            goBackward(.3, 20 - movedBack, -90.0 * alliance);
        }
    }
    public void positionOnBackdrop(String result, int alliance){
        double[] dists = getAprilTagDist(result);
        if(result.equals("Left")){
            //we want to be 2-3 inches left of the april tag
            if(dists[0] - 2.5 < 0){
                strafeRight(.35, -1 * (dists[0]-2.5), 5, -90.0*alliance);
            }else if(dists[0] - 2.5 > 0){
                strafeLeft(.35, dists[0]-2.5, 5, -90.0*alliance);
            }
        }else if(result.equals("Center")){
            //we want to be 1 inch left of the april tag
            if(dists[0] - 1 < 0){
                strafeRight(.35, -1 * (dists[0]-1), 5, -90.0*alliance);
            }else if(dists[0] - 2.5 > 0){
                strafeLeft(.35, dists[0]-1, 5, -90.0*alliance);
            }
        }else if(result.equals("Right")){
            //we want to be 2-3 inches right of the april tag
            if(dists[0] + 2.5 < 0){
                RobotLog.aa("Strafing", (-1*(dists[0] + 2.5)) + " inches robot-right, board-left");
                strafeRight(.35, -1 * (dists[0]+2.5), 5, -90.0*alliance);
            }else if(dists[0] - 2.5 > 0){
                RobotLog.aa("Strafing", (dists[0] + 2.5) + "inches robot-left, board-right");
                strafeLeft(.35, dists[0]+2.5, 5, -90.0*alliance);
            }
        }
        double inchesAway = 6.5;
        if(dists[1] - inchesAway > 0){
            sleep(100);
            goBackward(.3, dists[1]-inchesAway, -90.0*alliance);
        }else if(dists[1] - inchesAway < 0){
            sleep(100);
            goStraight(.3, dists[1]-inchesAway, -90.0*alliance);
        }
    }
}
