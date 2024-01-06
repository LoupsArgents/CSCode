package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        waitForStart();
        onRun(result, allianceNum);
    }
    public String onInit(int alliance){ //returns the team prop result
        initializeHardware();
        processor.setAlliance(alliance);
        processor.setMode(EverythingProcessor.ProcessorMode.PROP);
        setInitialPositions();
        String result = "Right"; //getPropResult();
        return result;
    }
    public void setInitialPositions(){
        wrist.setPosition(wristDownPos);
        sleep(100);
        closeClaw();
        sleep(500);
        wrist.setPosition(wristTuckedIn);
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
            goStraight(.4, 18, 0.0);
            if(alliance == 1){
                moveForwardLeft(.5, 4, 0.0);
            }else if(alliance == -1){
                moveForwardRight(.5, 4, 0.0);
            }
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
    }
    public void doYellowPixel(String result, int alliance) {
       getToBackdrop(result, alliance);
       absoluteHeading(.2, -90.0*alliance);
       sleep(500);
       positionOnBackdrop(result, alliance);
    }
    public void getToBackdrop(String result, int alliance){
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            goBackward(.3, 3);
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
            goBackward(.3, 5);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            goBackward(.3, 8, -90.0*alliance);
            if(alliance == 1){
                moveBackLeft(.35, 8, -90.0*alliance);
            }else{
                moveBackRight(.35, 8, -90.0*alliance);
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            goBackward(.3, 5);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            goBackward(.3, 20);
            sleep(250);
            if(alliance == 1){
                strafeLeft(.35, 5, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeRight(.35, 5, 5, -90.0*alliance);
            }
        }
    }
    public void positionOnBackdrop(String result, int alliance){
        double[] dists = getAprilTagDist(result);
        if(result.equals("Left")){
            //we want to be 2-3 inches left of the april tag
            if(dists[0] - 2.5 < 0){
                strafeRight(.35, dists[0]-2.5, 5, -90.0*alliance);
            }else if(dists[0] - 2.5 > 0){
                strafeLeft(.35, dists[0]-2.5, 5, -90.0*alliance);
            }
        }else if(result.equals("Center")){
            //we want to be 1 inch left of the april tag
            if(dists[0] - 1 < 0){
                strafeRight(.35, dists[0]-1, 5, -90.0*alliance);
            }else if(dists[0] - 2.5 > 0){
                strafeLeft(.35, dists[0]-1, 5, -90.0*alliance);
            }
        }else if(result.equals("Right")){
            //we want to be 2-3 inches right of the april tag
            if(dists[0] + 2.5 < 0){
                strafeRight(.35, dists[0]+2.5, 5, -90.0*alliance);
            }else if(dists[0] - 2.5 > 0){
                strafeLeft(.35, dists[0]+2.5, 5, -90.0*alliance);
            }
        }
        if(dists[1] - 10 > 0){
            sleep(100);
            goBackward(.3, dists[1]-10, -90.0*alliance);
        }else if(dists[1] - 10 < 0){
            sleep(100);
            goStraight(.3, dists[1]-10, -90.0*alliance);
        }
    }
}
