package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import java.time.ZonedDateTime;
import java.util.ArrayList;

@Disabled
public class CSPhillyAuto extends CSYorkDF {
    ArrayList<Double> rightAverages = new ArrayList<>();
    ArrayList<Double> centerAverages = new ArrayList<>();
    ArrayList<Double> leftAverages = new ArrayList<>();
    boolean parkingNearWall = false;

    int pixelsOnStack = 5;

    public void runOpMode(){}
    public void doRun(String alliance, boolean isNear){ //1 is blue, -1 is red
        int allianceNum = 1;
        if(alliance.equals("Blue")) allianceNum = 1;
        if(alliance.equals("Red")) allianceNum = -1;
        String result = onInit(allianceNum, isNear);
        //waitForStart();
        onRun(result, allianceNum, isNear);
    }
    public String onInit(int alliance, boolean isNear){ //returns the team prop result
        initializeHardware();
        processor.setAlliance(-alliance);
        processor.setIsStackMode(true);
        processor.setMode(EverythingProcessor.ProcessorMode.PROP);
        ArrayList<Double> rightAverages = new ArrayList<>();
        ArrayList<Double> leftAverages = new ArrayList<>();
        double leftAvg = 0.0;
        double rightAvg = 0.0;
        double camLeftInitial = 0.0;
        double camCenterInitial = 0.0;
        double camRightInitial = 0.0;
        setInitialPositions();
        boolean doneTheThing = false;
        while(opModeInInit()){
            if(isNear){
                if(!parkingNearWall){
                    telemetry.addData("Parking", "Away from wall");
                    telemetry.addLine("Press X on Gamepad 1 to switch to parking near wall");
                }else{
                    telemetry.addData("Parking", "Near wall");
                    telemetry.addLine("Press B on Gamepad 1 to switch to parking away from wall");
                }
            }
            telemetry.addData("Alliance", alliance);
            if(gamepad1.x) parkingNearWall = true;
            if(gamepad1.b) parkingNearWall = false;
            if(portal.getCameraState() == VisionPortal.CameraState.STREAMING && !doneTheThing){
                ZonedDateTime dt = ZonedDateTime.now();
                String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
                portal.saveNextFrameRaw("YorkAutoPropInitial " + time);
                camLeftInitial = processor.getLeftVal();
                camCenterInitial = processor.getCenterVal();
                camRightInitial = processor.getRightVal();
                RobotLog.aa("CamLeftInitial", String.valueOf(camLeftInitial));
                RobotLog.aa("CamRightInitial", String.valueOf(camRightInitial));
                RobotLog.aa("CamCenterInitial", String.valueOf(camCenterInitial));
                doneTheThing = true;
            }
            if(doneTheThing) telemetry.addData("Status", "Now you can put the prop down");
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
            telemetry.addData("Left", leftAvg);
            telemetry.addData("Right", rightAvg);
            telemetry.update();
        }
        waitForStart();
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("YorkAutoPropFinal " + time);
        String result = getPropResult(leftAvg, rightAvg, processor.getResult(camLeftInitial, camCenterInitial, camRightInitial));
        sleep(500);
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
        cameraBar.setPosition(camUsePos);
        sleep(500);
        telemetry.addData("Status", "Positions set");
    }
    public void onRun(String result, int alliance, boolean isNear){
        cameraBar.setPosition(camOutOfWay);
        //sleep(500);
        doPurplePixel(result, alliance, isNear);
        if(isNear) {
            doYellowPixel(result, alliance);
            cycle(result, alliance);
            //park(alliance, result, parkingNearWall); //once I get this figured out
        }
        //left out because cycling is incomplete
        //cycle(result, alliance);

    }
    public void doPurplePixel(String result, int alliance, boolean isNear){
        if((result.equals("Left") && ((alliance == 1 && isNear) || (alliance == -1 && !isNear))) || (result.equals("Right") && ((alliance == -1 && isNear) || (alliance == 1 && !isNear)))){
            double inchesMoved = 0.0;
            if(result.equals("Left")){
                inchesMoved = moveForwardLeft(.5, 10.5, 0.0);
            }else if(result.equals("Right")){
                inchesMoved = moveForwardRight(.5, 10.5, 0.0);
            }
            wrist.setPosition(wristAlmostDown);
            activateBackCamera();
            RobotLog.aa("Moved", Double.toString(inchesMoved));
            goStraight(.5, 16-inchesMoved, 0.0); //power was .3, then .4
            openLowerClaw();
            sleep(500);
        }else if(result.equals("Center")){
            //it is roughly 25 inches to the spike mark, so.
            double toSubtract = 0.0;
            if((alliance == 1 && isNear) || (alliance == -1 && !isNear)){
                toSubtract = moveForwardLeft(.55, 4, 0.0);
            }else if((alliance == -1 && isNear) || (alliance == 1 && !isNear)){
                toSubtract = moveForwardRight(.55, 4, 0.0);
            }
            wrist.setPosition(wristAlmostDown);
            RobotLog.aa("Subtracting", String.valueOf(toSubtract));
            activateBackCamera();
            RobotLog.aa("HeadingAfterDiagonal", String.valueOf(newGetHeading()));
            goStraight(.5, 22.5-toSubtract, 0.0); //used to be 25-toSubtract; power used to be .4
            openLowerClaw();
            sleep(500);
        }else if((result.equals("Right") && ((alliance == 1 && isNear) || (alliance == -1 && !isNear))) || (result.equals("Left") && ((alliance == -1 && isNear) || (alliance == 1 && !isNear)))){
            goStraight(.5, 15, 0.0);//power used to be .4
            wrist.setPosition(wristAlmostDown);
            activateBackCamera();
            double heading;
            if(isNear){
                heading = -45.0 * alliance;
            }else{
                heading = 45.0*alliance;
            }
            absoluteHeading(.4, heading);
            absoluteHeading(.2, heading);
            goStraight(.5, 2, heading); //power used to be .4
            openLowerClaw();
            sleep(500);
        }
    }
    public void doYellowPixel(String result, int alliance) {
        getToBackdrop(result, alliance);
        arm1.setPosition(arm1ScoringPos);
        wrist.setPosition(wristScoringPos);
        openLowerClaw();
        absoluteHeading(.2, -90.0*alliance);
        sleep(500);
        positionOnBackdrop(result, alliance);
        sleep(500);
        openUpperClaw();
        sleep(500);
    }
    public void getToBackdrop(String result, int alliance){
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            goBackward(.5, 3, 0.0 * alliance);
            closeLowerClaw();
            wrist.setPosition(wristAlmostDown);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            arm1.setPosition(armAlmostUp);
            goBackward(.5, 12, -90.0*alliance);
            cameraBar.setPosition(camTuckedIn);
            if(alliance == 1){
                //strafe left
                strafeLeft(.55, 7, 5, -90.0 * alliance);
            }else if(alliance == -1){
                //strafe right
                strafeRight(.55, 7, 5, -90.0 * alliance);
            }
        }else if(result.equals("Center")){
            goBackward(.5, 5, 0.0);
            closeLowerClaw();
            wrist.setPosition(wristAlmostDown);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            arm1.setPosition(armAlmostUp);
            goBackward(.5, 8, -90.0*alliance);
            cameraBar.setPosition(camTuckedIn);
            if(alliance == 1){
                moveBackLeft(.55, 8, -90.0*alliance);
            }else{
                moveBackRight(.55, 8, -90.0*alliance);
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            goBackward(.5, 6, -45.0*alliance);
            closeLowerClaw();
            wrist.setPosition(wristAlmostDown);
            absoluteHeading(.4, -90.0 * alliance);
            arm1.setPosition(armAlmostUp);
            absoluteHeading(.2, -90.0 * alliance);
            //so we want to have a net move of 10 inches strafe, 20 inches backward
            //goBackward(.3, 20, -90.0 * alliance);
            sleep(250);
            cameraBar.setPosition(camTuckedIn);
            double movedBack = 0.0;
            if(alliance == 1){
                //strafeLeft(.35, 5, 5, -90.0*alliance);
                movedBack = Math.abs(moveBackLeft(.6, 10, -90.0*alliance)); //powers were .55
            }else if(alliance == -1){
                //strafeRight(.35, 5, 5, -90.0*alliance);
                movedBack = Math.abs(moveBackRight(.6, 10, -90.0*alliance));
            }
            sleep(100);
            goBackward(.5, 20 - movedBack, -90.0 * alliance);
        }
    }
    public void positionOnBackdrop(String result, int alliance){
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("YorkAutoNearBoard " + time);
        double[] dists = getAprilTagDist(result);
        if(result.equals("Left")){
            //we want to be left of the april tag
            if(dists[0] - 1.5 < 0){
                strafeRight(.35, -1 * (dists[0]-1.5), 5, -90.0*alliance); //powers were .35
            }else if(dists[0] - 1.5 > 0){
                strafeLeft(.35, dists[0]-1.5, 5, -90.0*alliance);
            }
        }else if(result.equals("Center")){
            //we want to be 1 inch left of the april tag
            if(dists[0] - 1 < 0){
                strafeRight(.35, -1 * (dists[0]-1), 5, -90.0*alliance);
            }else if(dists[0] - 1 > 0){
                strafeLeft(.35, dists[0]-1, 5, -90.0*alliance);
            }
        }else if(result.equals("Right")){
            //we want to be 2-3 inches right of the april tag
            if(dists[0] + 1.5 < 0){
                RobotLog.aa("Strafing", (-1*(dists[0] + 1.5)) + " inches robot-right, board-left");
                strafeRight(.35, -1 * (dists[0]+1.5), 5, -90.0*alliance);
            }else if(dists[0] + 1.5 > 0){
                RobotLog.aa("Strafing", (dists[0] + 1.5) + "inches robot-left, board-right");
                strafeLeft(.35, dists[0]+1.5, 5, -90.0*alliance);
            }
        }
        double inchesAway = 6.25;
        if(dists[1] - inchesAway > 0){
            sleep(100);
            goBackward(.3, dists[1]-inchesAway, -90.0*alliance); //powers were .3
        }else if(dists[1] - inchesAway < 0){
            sleep(100);
            goStraight(.3, dists[1]-inchesAway, -90.0*alliance);
        }
    }
    public void cycle(String result, int alliance){
        //here, result = what part of the board you're coming from
        getToStack(result, alliance);
        getBackToBoard(result, alliance);
    }
    public void getToStack(String result, int alliance){
        //if we started on the side closest to the wall, it's 35 inches
        //6 inches between april tags, and remember to factor in the adjustments made
        double armPosForCycle;
        if(pixelsOnStack == 5) armPosForCycle = armStack45Pos;
        else armPosForCycle = armStack23Pos;
        goStraight(.55, 5, -90.0*alliance); //power was .4
        arm1.setPosition(armAlmostDown);
        wrist.setPosition(wristAlmostDown);
        double inchesMoved = 0.0;
        if(result.equals("Left")){
            if(alliance == 1){
                strafeLeft(.6, 18, 5 ,-90.0 * alliance);
            }else if(alliance == -1){
                strafeRight(.6, 18, 5 ,-90.0 * alliance); //powers on these were .4
            }
        }else if(result.equals("Center")){
            if(alliance == 1){
                strafeLeft(.6, 10, 5 ,-90.0 * alliance);
            }else if(alliance == -1){
                strafeRight(.6, 10, 5 ,-90.0 * alliance); //powers were .4
            }
        }else if(result.equals("Right")){
            if(alliance == 1){
                strafeLeft(.6, 4, 5 ,-90.0 * alliance);
            }else if(alliance == -1){
                strafeRight(.6, 4, 5 ,-90.0 * alliance); //powers were .4
            }
        }
        activateFrontCamera();
        arm1.setPosition(arm1DownPos);
        wrist.setPosition(wristDownPos);
        if(alliance == 1){
            inchesMoved = moveForwardLeft(.65, 12, -90.0*alliance);
        }else if(alliance == -1) {
            inchesMoved = moveForwardRight(.65, 12, -90.0 * alliance); //powers were .5
        }
        processor.setMode(EverythingProcessor.ProcessorMode.PIXEL);
        cameraBar.setPosition(camUsePos);
        goStraight(.6, 50-inchesMoved, -90.0*alliance); //power was .4
        wrist.setPosition(wristAlmostDown);
        arm1.setPosition(armPosForCycle);
        // inchesMoved = 0.0;
        //if(alliance == 1){
        //  inchesMoved = moveForwardRight(.6, 3, -90.0*alliance);
        //}else if(alliance == -1){
        //  inchesMoved = moveForwardLeft(.6, 3, -90.0*alliance);
        //}
        goStraight(.6, 32, -90.0*alliance); //power was .4
        wrist.setPosition(wristStack45Pos);
        sleep(500);
        arm1.setPosition(armPosForCycle); //to make sure it stays there
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("YorkAutoNearStack " + time);
        centerOnClosestStack(processor);
        sleep(500);
        pixelsOnStack -= 2;
    }
    public void getBackToBoard(String result, int alliance){
        cameraBar.setPosition(camTuckedIn);
        activateBackCamera();
        goBackward(.6, 70, -90.0*alliance);
        wrist.setPosition(wristAlmostDown);
        arm1.setPosition(armAlmostUp);
        goBackward(.6, 15, -90.0*alliance);
        if(alliance == 1){
            strafeRight(.6, 20, 5, -90.0*alliance);
        }else if(alliance == -1){
            strafeLeft(.6, 20, 5, -90.0*alliance);
        }
        wrist.setPosition(wristScoringPos);
        arm1.setPosition(arm1ScoringPos);
        sleep(500);
        if(alliance == 1){
            positionOnBackdrop("Right", alliance);
        }else if(alliance == -1){
            positionOnBackdrop("Left", alliance);
        }
        liftIdealPos = .1;
        while(Math.abs(liftIdealPos - liftPos) > .05){
            liftWithinLoop();
        }
        openLowerClaw();
        sleep(500);
        //sleep(200);
        openUpperClaw();
        cameraBar.setPosition(camOutOfWay);
        sleep(200);
        closeClaw();
        sleep(300);
        liftIdealPos = liftInitial;
        while(Math.abs(liftIdealPos - liftPos) > .05){
            liftWithinLoop();
        }
        arm1.setPosition(armAlmostDown);
        wrist.setPosition(wristAlmostDown);
        sleep(1000);
        arm1.setPosition(arm1DownPos);
        wrist.setPosition(wristDownPos);
    }
    public void park(int alliance, String result, boolean parkingNearWall){
        RobotLog.aa("Status", "Started parking");
        arm1.setPosition(armAlmostDown);
        wrist.setPosition(wristAlmostDown);
        if(alliance == -1){
            if(parkingNearWall){
                //~20 inches from the closest to the wall
                RobotLog.aa("Status", "Parking near wall");
                if(result.equals("Left")){
                    strafeLeft(.6, 31, 5, -90.0*alliance);
                }else if(result.equals("Center")){
                    strafeLeft(.6, 24, 5, -90.0*alliance);
                }else{
                    strafeLeft(.6, 18, 5, -90.0*alliance);
                }
            }else{
                RobotLog.aa("Status", "Parking away from wall");
                if(result.equals("Left")){
                    strafeRight(.6, 14, 5, -90.0*alliance);
                }else if(result.equals("Center")){
                    strafeRight(.6, 22, 5, -90.0*alliance);
                }else{
                    strafeRight(.6, 30, 5, -90.0*alliance);
                }
            }
        }else if(alliance == 1){
            RobotLog.aa("Status", "Hey we're on the blue alliance");
            if(parkingNearWall){
                //~20 inches from the closest to the wall
                RobotLog.aa("Status", "Parking near wall");
                if(result.equals("Left")){
                    strafeRight(.6, 18, 5, -90.0*alliance);
                }else if(result.equals("Center")){
                    strafeRight(.6, 24, 5, -90.0*alliance);
                }else{
                    strafeRight(.6, 31, 5, -90.0*alliance);
                }
            }else{
                RobotLog.aa("Status", "Parking away from wall");
                if(result.equals("Left")){
                    strafeLeft(.6, 30, 5, -90.0*alliance);
                }else if(result.equals("Center")){
                    strafeLeft(.6, 22, 5, -90.0*alliance);
                }else{
                    strafeLeft(.6, 14, 5, -90.0*alliance);
                }
            }
        }
        arm1.setPosition(arm1DownPos);
        wrist.setPosition(wristDownPos);
        sleep(1000);
    }
}
