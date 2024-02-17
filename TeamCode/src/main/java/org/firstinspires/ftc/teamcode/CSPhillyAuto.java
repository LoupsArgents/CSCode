package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.vision.VisionPortal;

import java.time.ZonedDateTime;
import java.util.ArrayList;

@Disabled
public class CSPhillyAuto extends CSYorkDF {
    ArrayList<Double> rightAverages = new ArrayList<>();
    ArrayList<Double> centerAverages = new ArrayList<>();
    ArrayList<Double> leftAverages = new ArrayList<>();
    boolean cycle = false;

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
        boolean streaming = false;
        long timeStreamingDetected = 0;
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
            if(cycle){
                telemetry.addData("Cycling", "True");
                telemetry.addLine("Press Y on Gamepad 1 to switch to not cycling");
            }else{
                telemetry.addData("Cycling", "False");
                telemetry.addLine("Press A on Gamepad 1 to switch to cycling");
            }
            if(gamepad1.a) cycle = true;
            if(gamepad1.y) cycle = false;
            telemetry.addData("Alliance", alliance);
            if(gamepad1.x) parkingNearWall = true;
            if(gamepad1.b) parkingNearWall = false;
            if(portal.getCameraState() == VisionPortal.CameraState.STREAMING && !streaming){
                timeStreamingDetected = System.currentTimeMillis();
                streaming = true;
            }
            long nowTime = System.currentTimeMillis();
            if (!doneTheThing && streaming && (nowTime - timeStreamingDetected > 1000)) {
                ZonedDateTime dt = ZonedDateTime.now();
                String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
                portal.saveNextFrameRaw("PropTestingInitial " + time);
                camLeftInitial = processor.getLeftVal();
                camCenterInitial = processor.getCenterVal();
                camRightInitial = processor.getRightVal();

                RobotLog.aa("CamLeftInitial", String.valueOf(camLeftInitial));
                RobotLog.aa("CamRightInitial", String.valueOf(camRightInitial));
                RobotLog.aa("CamCenterInitial", String.valueOf(camCenterInitial));
                doneTheThing = true;
            }
            if(doneTheThing){
                telemetry.addData("Status", "Now you can put the prop down");
                telemetry.addData("CamLeftInitial", camLeftInitial);
                telemetry.addData("CamCenterInitial", camCenterInitial);
                telemetry.addData("CamRightInitial", camRightInitial);
            }
            telemetry.update();
        }
        waitForStart();
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("PhillyAutoPropFinal " + time);
        String result = getPropResult(leftAvg, rightAvg, processor.getResult(camLeftInitial, camCenterInitial, camRightInitial));
        sleep(500);
        return result;
    }
    public void setInitialPositions(){
        //wrist slightly up, arm slightly up, flip wrist, arm down
        endStop.setPosition(endStopOutOfWayPos);
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
        sleep(1000);
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
            if(cycle) {
                cycle(result, alliance);
                String s;
                if (alliance == 1) s = "Right";
                else s = "Left";
                cycle(s, alliance);
                goStraight(.7, 2, -90.0 * alliance);
                liftIdealPos = liftInitial;
                while (Math.abs(liftIdealPos - liftPos) > .005) {
                    liftWithinLoop();
                }
                arm1.setPosition(armAlmostDown);
                wrist.setPosition(wristAlmostDown);
                sleep(1000);
                arm1.setPosition(arm1DownPos);
                wrist.setPosition(wristDownPos);
            }else park(alliance, result, parkingNearWall); //once I get this figured out
        }else{
            getToBoardFromFar(result, alliance);
            openLowerClaw();
            sleep(500);
            String r = result + "Center";
            positionOnBackdrop(r, alliance, 1);
            sleep(500);
            positionOnBackdrop(r, alliance, 2);
            sleep(500);
            openUpperClaw();
            sleep(500);
            goStraight(.5, 3, -90.0 * alliance);
            closeClaw();
            liftIdealPos = liftInitial;
            while (Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()) {
                liftWithinLoop();
            }
            arm1.setPosition(armAlmostDown);
            wrist.setPosition(wristAlmostDown);
            sleep(1000);
            arm1.setPosition(arm1DownPos);
            wrist.setPosition(wristDownPos);
            sleep(1000);
        }
        //left out because cycling is incomplete
        //cycle(result, alliance);

    }
    public void doPurplePixel(String result, int alliance, boolean isNear){
        if((result.equals("Left") && ((alliance == 1 && isNear) || (alliance == -1 && !isNear))) || (result.equals("Right") && ((alliance == -1 && isNear) || (alliance == 1 && !isNear)))){
            double inchesMoved = 0.0;
            if(result.equals("Left")){
                inchesMoved = moveForwardLeft(.8, 9, 0.0); //power was .5, then .6; was 10 inches
            }else if(result.equals("Right")){
                inchesMoved = moveForwardRight(.8, 9, 0.0);
            }
            wrist.setPosition(wristAlmostDown);
            activateBackCamera();
            RobotLog.aa("Moved", Double.toString(inchesMoved));
            goStraight(.5, 13-inchesMoved, 0.0); //power was .3, then .4; used to be 16-inchesMoved
            openLowerClaw();
            sleep(500);
        }else if(result.equals("Center")){
            //it is roughly 25 inches to the spike mark, so.
            double toSubtract = 0.0;
            if((alliance == 1 && isNear) || (alliance == -1 && !isNear)){
                toSubtract = moveForwardLeft(.6, 4, 0.0); //power was .55
            }else if((alliance == -1 && isNear) || (alliance == 1 && !isNear)){
                toSubtract = moveForwardRight(.6, 4, 0.0);
            }
            wrist.setPosition(wristAlmostDown);
            RobotLog.aa("Subtracting", String.valueOf(toSubtract));
            activateBackCamera();
            RobotLog.aa("HeadingAfterDiagonal", String.valueOf(newGetHeading()));
            goStraight(.6, 19.5-toSubtract, 0.0); //used to be 25-toSubtract, then 22.5-, then 20.5-; power used to be .4 then .5
            openLowerClaw();
            sleep(500);
        }else if((result.equals("Right") && ((alliance == 1 && isNear) || (alliance == -1 && !isNear))) || (result.equals("Left") && ((alliance == -1 && isNear) || (alliance == 1 && !isNear)))){
            goStraight(.6, 11, 0.0);//power used to be .4, then .5; used to be 15 inches before pathing change
            goStraight(.4, 5, 0.0);
            goStraight(.3, 5, 0.0);
            wrist.setPosition(wristAlmostDown);
            activateBackCamera();
            double heading;
            if(isNear){
                heading = -90.0 * alliance;
            }else{
                heading = 90.0*alliance;
            }
            absoluteHeading(.4, heading);
            absoluteHeading(.2, heading);
            //goStraight(.5, 2, heading); //power used to be .4 (from old pathing, may be necessary
            openLowerClaw();
            sleep(500);
        }
    }
    public void doYellowPixel(String result, int alliance) {
        getToBackdrop(result, alliance);
        arm1.setPosition(arm1ScoringPos);
        wrist.setPosition(wristScoringPos);
        openLowerClaw();
        //absoluteHeading(.2, -90.0*alliance);
        sleep(500);
        positionOnBackdrop(result, alliance, 1);
        sleep(500);
        positionOnBackdrop(result, alliance, 2);
        sleep(500);
        openUpperClaw();
        sleep(500);
    }
    public void getToBackdrop(String result, int alliance){
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            goBackward(.6, 3, 0.0 * alliance); //power was .5
            closeLowerClaw();
            wrist.setPosition(wristAlmostDown);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            arm1.setPosition(armAlmostUp);
            goBackward(.7, 4, -90.0*alliance); //power was .5, then .6
            double inchesMoved = 0.0;
            if(alliance == 1){
                //move left
                inchesMoved = Math.abs(moveBackLeft(.75, 7, -90.0 * alliance)); //powers were .55, then .65
            }else if(alliance == -1){
                //strafe right
                inchesMoved = Math.abs(moveBackRight(.75, 7, -90.0 * alliance));
            }
            if(8 - inchesMoved > 0) {
                goBackward(.7, 8 - inchesMoved);
            }
            cameraBar.setPosition(camTuckedIn);
        }else if(result.equals("Center")){
            goBackward(.7, 3, 0.0); //power was .5, then .6; was 5 inches
            closeLowerClaw();
            wrist.setPosition(wristAlmostDown);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            arm1.setPosition(armAlmostUp);
            goBackward(.7, 8, -90.0*alliance); //power was .5, then .6
            cameraBar.setPosition(camTuckedIn);
            if(alliance == 1){
                moveBackLeft(.75, 8, -90.0*alliance); //powers were .55, then .65
            }else{
                moveBackRight(.75, 8, -90.0*alliance);
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            //total ~25 inches back?
            goBackward(.7, 6, -90.0 * alliance);
            closeLowerClaw();
            wrist.setPosition(wristAlmostDown);
            arm1.setPosition(armAlmostUp);
            goBackward(.7, 15, -90.0*alliance); //was 19
            cameraBar.setPosition(camTuckedIn);
            /*goBackward(.7, 6, -45.0*alliance); //power was .5, then .6
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
                movedBack = Math.abs(moveBackLeft(.75, 10, -90.0*alliance)); //powers were .55, then .6, then .7
            }else if(alliance == -1){
                //strafeRight(.35, 5, 5, -90.0*alliance);
                movedBack = Math.abs(moveBackRight(.75, 10, -90.0*alliance));
            }
            sleep(100);
            goBackward(.7, 18 - movedBack, -90.0 * alliance); //power was .5, then .6; inches was 20*/
        }
    }
    public void positionOnBackdrop(String result, int alliance, int attempt){ //attempt 1 is initial pass, attempt 2 is second pass
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("PhillyAutoNearBoard " + time);
        String res = "";
        if(result.equals("LeftCenter")){
            res = "Left";
        }else if(result.equals("CenterCenter")){
            res = "Center";
        }else if(result.equals("RightCenter")){
            res = "Right";
        }
        double[] dists = getAprilTagDist(res);
        if(dists[0] == 0.0 && dists[1] == 0.0){
            if(attempt == 2) {
                RobotLog.aa("Status", "April tag not working -- ramming board");
                goBackward(.3, 6, -90.0 * alliance);
                return;
            }else{
                RobotLog.aa("Status", "April tag failed; waiting a long time");
                liftIdealPos = liftInitial;
                goStraight(.4, 3);
                while(Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()){
                  liftWithinLoop();
                }
                arm1.setPosition(armAlmostDown);
                wrist.setPosition(wristAlmostDown);
                sleep(1000);
                arm1.setPosition(arm1DownPos);
                wrist.setPosition(wristDownPos);
                sleep(30000);
            }
        }
        if(result.equals("Left")){
            //we want to be left of the april tag
            if(dists[0] + 1 < 0){ // used to be -1.5, then +2
                strafeRight(.35, -1 * (dists[0] + 1), 5, -90.0*alliance); //powers were .35
            }else if(dists[0] + 1 > 0){
                strafeLeft(.35, dists[0] + 1, 5, -90.0*alliance);
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
            //thing added used to be 1.5; that was too much
            //used to be -2
            if(dists[0] - 1 < 0){ //used to be +1.25
                RobotLog.aa("Strafing", (-1*(dists[0] - 1)) + " inches robot-right, board-left");
                strafeRight(.35, -1 * (dists[0] - 1), 5, -90.0*alliance);
            }else if(dists[0] - 1 > 0){
                RobotLog.aa("Strafing", (dists[0] - 1) + " inches robot-left, board-right");
                strafeLeft(.35, dists[0] - 1, 5, -90.0*alliance);
            }
        }else if(result.equals("LeftCenter")){
            if(dists[0] < 0){
                strafeRight(.35, -1 * (dists[0]), 5, -90.0*alliance); //powers were .35
            }else if(dists[0] > 0){
                strafeLeft(.35, dists[0], 5, -90.0*alliance);
            }
        }else if(result.equals("CenterCenter")){
            if(dists[0] < 0){
                strafeRight(.35, -1 * (dists[0]), 5, -90.0*alliance);
            }else if(dists[0] > 0){
                strafeLeft(.35, dists[0], 5, -90.0*alliance);
            }
        }else if(result.equals("RightCenter")){
            if(dists[0] < 0){
                RobotLog.aa("Strafing", (-1*(dists[0])) + " inches robot-right, board-left");
                strafeRight(.35, -1 * (dists[0]), 5, -90.0*alliance);
            }else if(dists[0] - 1 > 0){
                RobotLog.aa("Strafing", (dists[0]) + " inches robot-left, board-right");
                strafeLeft(.35, dists[0], 5, -90.0*alliance);
            }
        }
        double inchesAway; //used to be 6.25
        if(attempt == 1) inchesAway = 12;
        else inchesAway = 5.5; //used to be 6
        if(dists[1] - inchesAway > 0){
            sleep(100);
            RobotLog.aa("GoingBackward", (dists[1]-inchesAway) + " inches");
            goBackward(.3, dists[1]-inchesAway, -90.0*alliance); //powers were .3
        }else if(dists[1] - inchesAway < 0){
            sleep(100);
            RobotLog.aa("GoingForward", ((dists[1]-inchesAway) * -1) + " inches");
            goStraight(.3, -1 * (dists[1]-inchesAway), -90.0*alliance);
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
        double endStopPosForCycle;
        double wristPosForCycle;
        if(pixelsOnStack == 5){
            armPosForCycle = armStack45Pos;
            endStopPosForCycle = endStop45Pos;
            wristPosForCycle = wristStack45Pos;
        }else{
            armPosForCycle = armStack23Pos;
            endStopPosForCycle = endStop23Pos;
            wristPosForCycle = wristStack23Pos;
        }
        goStraight(.75, 2, -90.0*alliance); //power was .4, then .55, then .65; increased distance from 5; was 7; was 5
        arm1.setPosition(armAlmostDown);
        wrist.setPosition(wristAlmostDown);
        liftIdealPos = liftInitial;
        double inchesMoved = 0.0;
       /* if(result.equals("Left")){
            if(alliance == 1){
                strafeLeft(.8, 14.75, 5 ,-90.0 * alliance);
            }else if(alliance == -1){
                strafeRight(.8, 14.75, 5 ,-90.0 * alliance); //powers on these were .4, then .6, then .7
            }
        }else if(result.equals("Center")){
            if(alliance == 1){
                strafeLeft(.8, 12, 5 ,-90.0 * alliance); //distance was 10
            }else if(alliance == -1){
                strafeRight(.8, 12, 5 ,-90.0 * alliance); //powers were .4, then .6, then .7
            }
        }else if(result.equals("Right")){
            if(alliance == 1){
                strafeLeft(.8, 7.5, 5 ,-90.0 * alliance);
            }else if(alliance == -1){
                strafeRight(.8, 7.5, 5 ,-90.0 * alliance); //powers were .4, then .6, then .7
            }
        }*/
        inchesMoved = 0.0;
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            if(alliance == 1){
                strafeLeft(.6, 25, 5, -90.0*alliance); //was 23.75 as a diagonal movement
            }else if(alliance == -1){
                strafeRight(.6, 25, 5, -90.0*alliance); //powers on these were .4, then .6, then .7
            }
            sleep(500);
        }else if(result.equals("Center")){
            if(alliance == 1){
                inchesMoved = moveForwardLeft(.85, 21, -90.0*alliance);
            }else if(alliance == -1){
                inchesMoved = moveForwardRight(.85, 21, -90.0*alliance); //powers were .4, then .6, then .7
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            if(alliance == 1){
                inchesMoved = moveForwardLeft(.85, 16.5, -90.0*alliance);
            }else if(alliance == -1){
                inchesMoved = moveForwardRight(.85, 16.5, -90.0*alliance); //powers were .4, then .6, then .7
            }
        }

        activateFrontCamera();
        //arm1.setPosition(arm1DownPos);
        //wrist.setPosition(wristDownPos);
        /*if(alliance == 1){
            inchesMoved = moveForwardLeft(.85, 8, -90.0*alliance); //distance used to be 12
        }else if(alliance == -1) {
            inchesMoved = moveForwardRight(.85, 8, -90.0 * alliance); //powers were .5, then .65, then .75
        }*/
        arm1.setPosition(arm1DownPos);
        wrist.setPosition(wristDownPos);
        processor.setMode(EverythingProcessor.ProcessorMode.PIXEL);
        goStraight(.8, 59-inchesMoved, -90.0*alliance); //power was .4, then .6, then .7; added 3 inches because other forward was less
        cameraBar.setPosition(camUsePos);
        wrist.setPosition(wristAlmostDown);
        arm1.setPosition(armPosForCycle);
        endStop.setPosition(endStopPosForCycle);
        // inchesMoved = 0.0;
        //if(alliance == 1){
        //  inchesMoved = moveForwardRight(.6, 3, -90.0*alliance);
        //}else if(alliance == -1){
        //  inchesMoved = moveForwardLeft(.6, 3, -90.0*alliance);
        //}
        goStraight(.8, 10, -90.0*alliance); //power was .4, then .6, then .7; inches was 32, then 25, but became too much
        goStraight(.6, 6, -90.0*alliance);
        goStraight(.4, 6, -90.0*alliance);
        wrist.setPosition(wristPosForCycle);
        absoluteHeading(.2, -90.0*alliance);
        sleep(500);
        arm1.setPosition(armStallAgainstStopPos); //to make sure it stays there
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("PhillyAutoNearStack " + time);
        centerOnClosestStack(processor);
        cameraBar.setPosition(camTuckedIn);
        sleep(250); //was 750
        //sleep(3000);
        arm1.setPosition(armPosForCycle - .01);
        goBackward(.4, .5,-90.0*alliance);
        endStop.setPosition(endStopOutOfWayPos);
        pixelsOnStack -= 2;
    }
    public void getToBoardFromFar(String result, int alliance){
        RobotLog.aa("Result", result);
        RobotLog.aa("Alliance", String.valueOf(alliance));
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            goBackward(.5, 4, 90.0 * alliance);
            closeLowerClaw();
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            if(alliance == 1){
                strafeLeft(.6, 22, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeRight(.6, 22, 5, -90.0*alliance);
            }
            sleep(500);
            goBackward(.6, 50, -90.0*alliance);
            wrist.setPosition(wristAlmostDown);
            arm1.setPosition(armAlmostUp);
            goBackward(.6, 22, -90.0*alliance); //was 25, then 20
            wrist.setPosition(wristScoringPos);
            arm1.setPosition(arm1ScoringPos);
            liftIdealPos = liftYellowPixelPos;
            sleep(500);
            if(alliance == 1){
                strafeRight(.6, 23, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeLeft(.6, 23, 5, -90.0*alliance);
            }
        }else if(result.equals("Center")){
            goBackward(.5, 2, 0.0);
            closeLowerClaw();
            sleep(500);
            if(alliance == 1) {
                strafeRight(.6, 10, 5, 0.0);
            }else if(alliance == -1){
                strafeLeft(.6, 10, 5, 0.0);
            }
            cameraBar.setPosition(camUsePos);
            sleep(500);
            goStraight(.5, 17, 0.0);
            sleep(500);
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            cameraBar.setPosition(camOutOfWay);
            sleep(500);
            if(alliance == 1) {
                strafeLeft(.5, 5, 5, -90.0 * alliance);
            }else if(alliance == -1){
                strafeRight(.5, 5, 5, -90.0*alliance);
            }
            goBackward(.6, 63, -90.0*alliance);
            wrist.setPosition(wristAlmostDown);
            arm1.setPosition(armAlmostUp);
            goBackward(.6, 22, -90.0*alliance); //was 25, then 20
            wrist.setPosition(wristScoringPos);
            arm1.setPosition(arm1ScoringPos);
            liftIdealPos = liftYellowPixelPos;
            sleep(500);
            if(alliance == 1){
                strafeRight(.6, 23, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeLeft(.6, 23, 5, -90.0*alliance);
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            goBackward(.5, 2, 0.0);
            sleep(500);
            if(alliance == 1){
                strafeLeft(.4, 10, 5, 0.0);
            }else {
                strafeRight(.4, 10, 5, 0.0);
            }
            closeLowerClaw();
            sleep(500);
            cameraBar.setPosition(camUsePos);
            goStraight(.5, 26, 0.0);
            sleep(500);
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            cameraBar.setPosition(camOutOfWay);
            sleep(500);
            if (alliance == 1) {
                strafeLeft(.5, 2.5, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeRight(.5, 2.5, 5, -90.0*alliance);
            }
            goBackward(.6, 45, -90.0*alliance);
            wrist.setPosition(wristAlmostDown);
            arm1.setPosition(armAlmostUp);
            goBackward(.6, 22, -90.0*alliance); //was 25, then 20
            wrist.setPosition(wristScoringPos);
            arm1.setPosition(arm1ScoringPos);
            liftIdealPos = liftYellowPixelPos;
            sleep(500);
            if(alliance == 1){
                strafeRight(.6, 18, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeLeft(.6, 18, 5, -90.0*alliance);
            }
        }
    }
    public void getBackToBoard(String result, int alliance){
        activateBackCamera();
        goBackward(.8, 64.5, -90.0*alliance); //power was .6, then .7; distance was 70 but became too close for apriltags
        wrist.setPosition(wristAlmostDown);
        arm1.setPosition(armAlmostUp);
        goBackward(.8, 12, -90.0*alliance); //power was .6, then .7; distance was 15 but started being too much bc speed increase
        liftIdealPos = .06;
        if(alliance == 1){
            strafeRight(.8, 13, 5, -90.0*alliance); //powers were .6, then .7; dists were 20 then 15
        }else if(alliance == -1){
            strafeLeft(.8, 13, 5, -90.0*alliance);
        }
        wrist.setPosition(wristScoringPos);
        arm1.setPosition(arm1ScoringPos);
        sleep(500);
        if(alliance == 1){
            positionOnBackdrop("Right", alliance, 1);
            sleep(500);
            positionOnBackdrop("Right", alliance, 2);

        }else if(alliance == -1){
            positionOnBackdrop("Left", alliance, 1);
            sleep(500);
            positionOnBackdrop("Left", alliance, 2);
        }
        /*liftIdealPos = .07;
        while(Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()){
            liftWithinLoop();
        }*/
        openLowerClaw();
        sleep(500);
        //sleep(200);
        liftIdealPos = .11;
        goStraight(.35, .5, -90.0*alliance);
        while(Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()){
            liftWithinLoop();
        }
        openUpperClaw();
        cameraBar.setPosition(camOutOfWay);
        sleep(500);
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
