package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.vision.VisionPortal;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;

@Disabled
public class CSPhillyAuto extends CSYorkDF {
    ArrayList<Double> rightAverages = new ArrayList<>();
    ArrayList<Double> centerAverages = new ArrayList<>();
    ArrayList<Double> leftAverages = new ArrayList<>();
    boolean cycle = false;

    boolean parkingNearWall = false;
    int pixelsOnStack = 5;
    int delay = 0;

    public void runOpMode(){}
    public void doRun(String alliance, boolean isNear, boolean isWall){ //1 is blue, -1 is red
        int allianceNum = 1;
        if(alliance.equals("Blue")) allianceNum = 1;
        if(alliance.equals("Red")) allianceNum = -1;
        String result = onInit(allianceNum, isNear);
        //waitForStart();
        onRun(result, allianceNum, isNear, isWall);
    }
    public String onInit(int alliance, boolean isNear){ //returns the team prop result
        initializeHardware();
        processor.setAlliance(-alliance);
        processor.setIsStackMode(true);
        processor.setMode(EverythingProcessor.ProcessorMode.PROP);
        double leftAvg = 0.0;
        double rightAvg = 0.0;
        double camLeftInitial = 0.0;
        double camCenterInitial = 0.0;
        double camRightInitial = 0.0;
        boolean streaming = false;
        long timeStreamingDetected = 0;
        setInitialPositions();
        boolean doneTheThing = false;
        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        while(opModeInInit()){
            if(cycle){
                telemetry.addData("Cycling", "True");
                telemetry.addLine("Press Y on Gamepad 1 to switch to not cycling");
            }else{
                telemetry.addData("Cycling", "False");
                telemetry.addLine("Press A on Gamepad 1 to switch to cycling");
            }
            if(gamepad1.a) cycle = true;
            if(gamepad1.y) cycle = false;
            telemetry.addData("DelayAfterPurplePixel", (delay/1000) + " seconds");
            telemetry.addLine("Use the up/down D-pad buttons on Gamepad 1 to change the delay in 1-second increments");
            if(gamepad1.dpad_up){
                if(!dpadUpPressed){
                    dpadUpPressed = true;
                    delay += 1000;
                }
            }else{
                dpadUpPressed = false;
            }
            if(gamepad1.dpad_down){
                if(!dpadDownPressed && delay > 0){
                    dpadDownPressed = true;
                    delay -= 1000;
                }
            }else{
                dpadDownPressed = false;
            }
            if(!parkingNearWall){
                telemetry.addData("Parking", "Away from wall");
                telemetry.addLine("Press X on Gamepad 1 to switch to parking near wall");
            }else{
                telemetry.addData("Parking", "Near wall");
                telemetry.addLine("Press B on Gamepad 1 to switch to parking away from wall");
            }
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
                portal.saveNextFrameRaw("PhillyAutoPropInitial " + time);
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
        armDown();
        endStop.setPosition(endStopOutOfWayPos);
        cameraBar.setPosition(camTuckedIn);
        closeClaw(); //see what the less-closed claw position is
        sleep(5000); //hopefully enough for people to get out of the way + not too much
        wrist.setPosition(wristAlmostDown);
        sleep(250);
        armAlmostDown();
        sleep(500);
        wrist.setPosition(wristTuckedIn);
        sleep(250);
        armDown();
        sleep(1000);
        cameraBar.setPosition(camUsePos);
        sleep(500);
        clawUp.setPosition(clawUpInitClose);
        clawDown.setPosition(clawDownInitClose);
        arm1.setPosition(arm1InitDownPos);
        telemetry.addData("Status", "Positions set");
    }
    public void onRun(String result, int alliance, boolean isNear, boolean isWall){
        closeClaw();
        armDown();
        cameraBar.setPosition(camOutOfWay);
        //sleep(500);
        doPurplePixel(result, alliance, isNear);
        if(isNear) {
            doYellowPixel(result, alliance);
        }else{
            long startTime = System.currentTimeMillis();
            long nowTime = System.currentTimeMillis();
            while(opModeIsActive() && (nowTime - startTime) < delay){
                nowTime = System.currentTimeMillis();
            }
            if(!isWall) {
                getToBoardFromFar(result, alliance);
            }else{
                getToBoardFromWall(result, alliance);
            }
            clawDown.setPosition(clawDownLessOpen); //openLowerClaw();
            sleep(500);
            String r = result + "Center";
            positionOnBackdrop(r, alliance, 1);
            sleep(500);
            positionOnBackdrop(r, alliance, 2);
            sleep(500);
            clawUp.setPosition(clawUpLessOpen); //openUpperClaw();
            long start = System.nanoTime();
            activateFrontCamera();
            long now = System.nanoTime();
            long elapsed = now - start;
            if((elapsed/Math.pow(10, 6)) < 500){
                sleep((500 - (int)(elapsed/Math.pow(10, 6))));
            }
            goStraight(.5, 3, -90.0 * alliance);
            closeClaw();
            liftIdealPos = liftInitial;
            liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);
            while (Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()) {
                liftWithinLoop();
            }
        }
        if(cycle) {
            cycle(result, alliance);
            goStraight(.7, 2, -90.0 * alliance);
            closeClaw();
            liftIdealPos = liftInitial;
            liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);
            while (Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()) {
                liftWithinLoop();
            }
            armAlmostDown();
            wrist.setPosition(wristAlmostDown);
            sleep(1000);
            armDown();
            wrist.setPosition(wristDownPos);
            String s;
            if(alliance == 1){
                if(!result.equals("Right")) {
                    s = "Right";
                }else{
                    s = "Center";
                }
            }else{
                if(!result.equals("Left")) {
                    s = "Left";
                }else{
                    s = "Center";
                }
            }
            park(alliance, s, parkingNearWall); //once I get this figured out
        }else{
            armAlmostDown();
            wrist.setPosition(wristAlmostDown);
            sleep(1000);
            armDown();
            wrist.setPosition(wristDownPos);
            park(alliance, result, parkingNearWall);
        }
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
            activateBackCamera();
            RobotLog.aa("Subtracting", String.valueOf(toSubtract));
            RobotLog.aa("HeadingAfterDiagonal", String.valueOf(newGetHeading()));
            goStraight(.6, 20-toSubtract, 0.0); //used to be 25-toSubtract, then 22.5-, then 20.5-, then 19.5-; power used to be .4 then .5
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
        armUp();
        wrist.setPosition(wristScoringPos);
        openLowerClaw();
        //absoluteHeading(.2, -90.0*alliance);
        sleep(500);
        positionOnBackdrop(result, alliance, 1);
        sleep(500);
        positionOnBackdrop(result, alliance, 2);
        sleep(500);
        openUpperClaw();
        long start = System.nanoTime();
        activateFrontCamera();
        long now = System.nanoTime();
        long elapsed = now - start;
        if((elapsed/Math.pow(10, 6)) < 500){
            sleep((500 - (int)(elapsed/Math.pow(10, 6))));
        }
    }
    public void getToBackdrop(String result, int alliance){
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            goBackward(.6, 3, 0.0 * alliance); //power was .5
            closeLowerClaw();
            wrist.setPosition(wristAlmostDown);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            armAlmostUp();
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
            goBackward(.7, 3.5, 0.0); //power was .5, then .6; was 5 inches, then 3 then 4
            closeLowerClaw();
            wrist.setPosition(wristAlmostDown);
            absoluteHeading(.4, -90.0 * alliance);
            absoluteHeading(.2, -90.0 * alliance);
            armAlmostUp();
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
            armAlmostUp();
            goBackward(.7, 15, -90.0*alliance); //was 19
            cameraBar.setPosition(camTuckedIn);
        }
    }
    public void positionOnBackdrop(String result, int alliance, int attempt){ //attempt 1 is initial pass, attempt 2 is second pass
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("PhillyAutoNearBoardFirst " + time);
        String res = result;
        if(result.equals("LeftCenter")){
            res = "Left";
        }else if(result.equals("CenterCenter")){
            res = "Center";
        }else if(result.equals("RightCenter")){
            res = "Right";
        }
        double[] dists = getAprilTagDist(res);
        RobotLog.aa("IdealPos", result);
        RobotLog.aa("Dists", Arrays.toString(dists));
        if(dists[0] == 0.0 && dists[1] == 0.0){
            if(attempt == 2) {
                RobotLog.aa("Status", "April tag not working -- ramming board");
                goBackward(.3, 5.75, -90.0 * alliance);
                return;
            }else{
                RobotLog.aa("Status", "Didn't see April tag on first positioning attempt -- moving and trying again");
                goBackward(.3, 2, -90.0*alliance);
                sleep(500); //CHANGE THIS BACK TO 500!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                dt = ZonedDateTime.now();
                time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
                portal.saveNextFrameRaw("PhillyAutoNearBoardTryAgain " + time);
                dists = getAprilTagDist(res);
                RobotLog.aa("TryingAgainDists", Arrays.toString(dists));
                if(dists[0] == 0.0 && dists[1] == 0.0){
                    liftIdealPos = liftInitial;
                    goStraight(.4, 3);
                    liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);
                    while(Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()){
                        liftWithinLoop();
                    }
                    armAlmostDown();
                    wrist.setPosition(wristAlmostDown);
                    sleep(1000);
                    armDown();
                    wrist.setPosition(wristDownPos);
                    sleep(30000);
                }
                /*RobotLog.aa("Status", "April tag failed; waiting a long time");
                liftIdealPos = liftInitial;
                goStraight(.4, 3);
                liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);
                while(Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()){
                  liftWithinLoop();
                }
                armAlmostDown();
                wrist.setPosition(wristAlmostDown);
                sleep(1000);
                armDown();
                wrist.setPosition(wristDownPos);
                sleep(30000);*/
            }
        }
        if(result.equals("Left")){
            //we want to be left of the april tag
            if(dists[0] + 1 < 0){ // used to be -1.5, then +2
                RobotLog.aa("Strafing", (-1*(dists[0] + 1)) + " inches robot-right, board-left");
                strafeRight(.35, -1 * (dists[0] + 1), 5, -90.0*alliance); //powers were .35
            }else if(dists[0] + 1 > 0){
                RobotLog.aa("Strafing", (dists[0] + 1) + " inches robot-left, board-right");
                strafeLeft(.35, dists[0] + 1, 5, -90.0*alliance);
            }
        }else if(result.equals("Center")){
            //we want to be 1 inch left of the april tag
            if(dists[0] - 1 < 0){
                RobotLog.aa("Strafing", (-1*(dists[0] - 1)) + " inches robot-right, board-left");
                strafeRight(.35, -1 * (dists[0]-1), 5, -90.0*alliance);
            }else if(dists[0] - 1 > 0){
                RobotLog.aa("Strafing", (dists[0] - 1) + " inches robot-left, board-right");
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
        long startTime = System.currentTimeMillis();
        long nowTime = System.currentTimeMillis();
        while(opModeIsActive() && (nowTime - startTime) < delay){
            nowTime = System.currentTimeMillis();
        }
        getBackToBoard(result, alliance);
    }
    public void getToStack(String result, int alliance){
        //if we started on the side closest to the wall, it's 35 inches
        //6 inches between april tags, and remember to factor in the adjustments made
        double endStopPosForCycle;
        double wristPosForCycle;
        if(pixelsOnStack == 5){
            endStopPosForCycle = endStop45Pos;
            wristPosForCycle = wristStack45Pos;
        }else{
            endStopPosForCycle = endStop23Pos;
            wristPosForCycle = wristStack23Pos;
        }
        goStraight(.75, 2, -90.0*alliance); //power was .4, then .55, then .65; increased distance from 5; was 7; was 5
        openClaw();
        armAlmostDown();
        wrist.setPosition(wristAlmostDown);
        liftIdealPos = liftInitial;
        double inchesMoved = 0.0;
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            if(alliance == 1){
                strafeLeft(.6, 25, 5, -90.0*alliance); //was 23.75 as a diagonal movement
            }else if(alliance == -1){
                strafeRight(.6, 25, 5, -90.0*alliance); //powers on these were .4, then .6, then .7
            }
            sleep(300); //was 500
        }else if(result.equals("Center")){
            if(alliance == 1){
                inchesMoved = moveForwardLeft(.85, 21, -90.0*alliance);
            }else if(alliance == -1){
                inchesMoved = moveForwardRight(.85, 21, -90.0*alliance); //powers were .4, then .6, then .7
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            if(alliance == 1){
                inchesMoved = moveForwardLeft(.85, 14.5, -90.0*alliance); //was .85
            }else if(alliance == -1){
                inchesMoved = moveForwardRight(.85, 14.5, -90.0*alliance); //powers were .4, then .6, then .7
            }
        }
        sleep(300);
        armDown();
        wrist.setPosition(wristDownPos);
        processor.setMode(EverythingProcessor.ProcessorMode.PIXEL);
        goStraight(.8, 59-inchesMoved, -90.0*alliance); //power was .4, then .6, then .7; added 3 inches because other forward was less
        cameraBar.setPosition(camUsePos);
        wrist.setPosition(wristAlmostDown);
        arm1.setPosition(arm1DownPos - .05); //used to be defined in terms of the stack levels; we'll see how this works.
        endStop.setPosition(endStopPosForCycle);
        goStraight(.6, 12, -90.0*alliance);
        goStraight(.4, 10, -90.0*alliance);
        wrist.setPosition(wristPosForCycle);
        //absoluteHeading(.2, -90.0*alliance);
        sleep(500);
        stallArm(); //to make sure it stays there
        ZonedDateTime dt = ZonedDateTime.now();
        String time = dt.getMonthValue() + "-" + dt.getDayOfMonth() + "-" + dt.getYear() + " " + dt.getHour() + "." + dt.getMinute() + "." + dt.getSecond();
        portal.saveNextFrameRaw("PhillyAutoNearStack " + time);
        centerOnClosestStack(processor);
        cameraBar.setPosition(camTuckedIn);
        long start = System.nanoTime();
        activateBackCamera();
        long now = System.nanoTime();
        long elapsed = now - start;
        if((elapsed/Math.pow(10, 6)) < 500){
            wait((500 - (int)(elapsed/Math.pow(10, 6))));
        }
        arm1.setPosition(arm1DownPos - .1);
        goBackward(.4, .5,-90.0*alliance);
        endStop.setPosition(endStopOutOfWayPos);
        pixelsOnStack -= 2;
    }
    public void getToBoardFromFar(String result, int alliance){
        RobotLog.aa("Result", result);
        RobotLog.aa("Alliance", String.valueOf(alliance));
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            goBackward(.5, 4, 90.0 * alliance);
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            closeLowerClaw();
            if(alliance == 1){
                strafeLeft(.7, 19, 5, -90.0*alliance); //power was .6, was 22in
            }else if(alliance == -1){
                strafeRight(.7, 19, 5, -90.0*alliance);
            }
            sleep(300); //was 500
            goBackward(.7, 50, -90.0*alliance); //power was .6
            wrist.setPosition(wristAlmostDown);
            armAlmostUp();
            goBackward(.7, 22, -90.0*alliance); //was 25, then 20; power was .6
            wrist.setPosition(wristScoringPos);
            armUp();
            liftIdealPos = liftYellowPixelPos;
            sleep(300); //was 500
            if(alliance == 1){
                strafeRight(.7, 23, 5, -90.0*alliance); //power was .6
            }else if(alliance == -1){
                strafeLeft(.7, 23, 5, -90.0*alliance);
            }
        }else if(result.equals("Center")){
            goBackward(.5, 2.5, 0.0); //was 2 then 3
            sleep(300); //was 500
            if(alliance == 1) {
                strafeRight(.7, 10, 5, 0.0); //power was .6
            }else if(alliance == -1){
                strafeLeft(.7, 10, 5, 0.0);
            }
            closeLowerClaw();
            cameraBar.setPosition(camUsePos);
            sleep(300); //was 500
            goStraight(.6, 15, 0.0); //power was .5, was 17 in
            sleep(300); //was 500
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            cameraBar.setPosition(camOutOfWay);
            sleep(300); //was 500
            if(alliance == 1) {
                strafeLeft(.5, 5, 5, -90.0 * alliance);
            }else if(alliance == -1){
                strafeRight(.5, 5, 5, -90.0*alliance);
            }
            goBackward(.7, 63, -90.0*alliance); //power was .6
            wrist.setPosition(wristAlmostDown);
            armAlmostUp();
            goBackward(.7, 22, -90.0*alliance); //was 25, then 20
            wrist.setPosition(wristScoringPos);
            armUp();
            liftIdealPos = liftYellowPixelPos;
            sleep(300); //was 500
            if(alliance == 1){
                strafeRight(.7, 23, 5, -90.0*alliance); //power was .6
            }else if(alliance == -1){
                strafeLeft(.7, 23, 5, -90.0*alliance);
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            goBackward(.5, 2, 0.0);
            sleep(300); //was 500
            if(alliance == 1){
                strafeLeft(.5, 10, 5, 0.0); //power was .4
            }else {
                strafeRight(.5, 10, 5, 0.0);
            }
            closeLowerClaw();
            sleep(300); //was 500
            cameraBar.setPosition(camUsePos);
            goStraight(.7, 23, 0.0); //power was .5, was 26 inches
            sleep(300); //was 500
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            cameraBar.setPosition(camOutOfWay);
            sleep(300); //was 500
            if (alliance == 1) {
                strafeLeft(.7, 2.5, 5, -90.0*alliance); //power was .5
            }else if(alliance == -1){
                strafeRight(.7, 2.5, 5, -90.0*alliance);
            }
            goBackward(.7, 45, -90.0*alliance); //power was .6
            wrist.setPosition(wristAlmostDown);
            armAlmostUp();
            goBackward(.7, 22, -90.0*alliance); //was 25, then 20; power was .6
            wrist.setPosition(wristScoringPos);
            armUp();
            liftIdealPos = liftYellowPixelPos;
            sleep(300); //was 500
            if(alliance == 1){
                strafeRight(.7, 16, 5, -90.0*alliance); //power was .6; was 18
            }else if(alliance == -1){
                strafeLeft(.7, 16, 5, -90.0*alliance);
            }
        }
    }
    public void getToBoardFromWall(String result, int alliance){
        if((result.equals("Left") && alliance == 1) || (result.equals("Right") && alliance == -1)){
            goBackward(.5, 4, 90.0*alliance);
            closeLowerClaw();
            sleep(300); //was 500
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            sleep(300); //was 500
            if(alliance == 1){
                strafeRight(.5, 22, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeLeft(.5, 22, 5, -90.0*alliance);
            }
            sleep(300); //was 500
            goBackward(.6, 55, -90.0*alliance);
            wrist.setPosition(wristAlmostDown);
            armAlmostUp();
            goBackward(.6, 18, -90.0*alliance);
            wrist.setPosition(wristScoringPos);
            armUp();
            liftIdealPos = liftYellowPixelPos;
            sleep(300); //was 500
            if(alliance == 1){
                strafeLeft(.5, 19, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeRight(.5, 19, 5, -90.0*alliance);
            }
        }else if(result.equals("Center")){
            goBackward(.5, 4.5, 0.0);
            closeLowerClaw();
            sleep(300); //was 500
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            sleep(300); //was 500
            if(alliance == 1){
                strafeRight(.5, 15, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeLeft(.5, 15, 5, -90.0*alliance);
            }
            sleep(300); //was 500
            goBackward(.6, 55, -90.0*alliance);
            wrist.setPosition(wristAlmostDown);
            armAlmostUp();
            goBackward(.6, 18, -90.0*alliance);
            wrist.setPosition(wristScoringPos);
            armUp();
            liftIdealPos = liftYellowPixelPos;
            sleep(300); //was 500
            if(alliance == 1){
                strafeLeft(.5, 19, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeRight(.5, 19, 5, -90.0*alliance);
            }
        }else if((result.equals("Right") && alliance == 1) || (result.equals("Left") && alliance == -1)){
            goBackward(.5, 4, 0.0);
            closeLowerClaw();
            sleep(300); //was 500
            absoluteHeading(.4, -90.0*alliance);
            absoluteHeading(.2, -90.0*alliance);
            sleep(300); //was 500
            if(alliance == 1){
                strafeRight(.5, 6.5, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeLeft(.5, 6.5, 5, -90.0*alliance);
            }
            sleep(300); //was 500
            goBackward(.6, 60, -90.0*alliance);
            wrist.setPosition(wristAlmostDown);
            armAlmostUp();
            goBackward(.6, 18, -90.0*alliance);
            wrist.setPosition(wristScoringPos);
            armUp();
            liftIdealPos = liftYellowPixelPos;
            sleep(300); //was 500
            if(alliance == 1){
                strafeLeft(.5, 19, 5, -90.0*alliance);
            }else if(alliance == -1){
                strafeRight(.5, 19, 5, -90.0*alliance);
            }
        }
    }
    public void getBackToBoard(String result, int alliance){
        goBackward(.8, 65.5, -90.0*alliance); //power was .6, then .7; distance was 70 but became too close for apriltags; was 54.5 then 55.5 then changed deceleration
        wrist.setPosition(wristAlmostDown);
        armAlmostUp();
        goBackward(.7, 15, -90.0*alliance); //was 18
        liftIdealPos = liftFirstWhitePixelPos;
        sleep(300); //was 500
        String res = "Right";
        if(alliance == -1){
            res = "Left";
            if(result.equals("Left")){
                res = "Center";
            }
        }else{
            if(result.equals("Right")){
                res = "Center";
            }
        }
        if((res.equals("Left") && alliance == 1) || (res.equals("Right") && alliance == -1)){
            if(alliance == 1){
                strafeRight(.7, 23, 5, -90.0*alliance); //powers were .6, then .7; dists were 20 then 15
            }else if(alliance == -1){
                strafeLeft(.7, 23, 5, -90.0*alliance);
            }
        }else if(res.equals("Center")){
            if(alliance == 1){
                strafeRight(.7, 23, 5, -90.0*alliance); //powers were .6, then .7; dists were 20 then 15
            }else if(alliance == -1){
                strafeLeft(.7, 23, 5, -90.0*alliance);
            }
        }else{
            if(alliance == 1){
                strafeRight(.7, 15, 5, -90.0*alliance); //powers were .6, then .7; dists were 20 then 15
            }else if(alliance == -1){
                strafeLeft(.7, 15, 5, -90.0*alliance);
            }
        }
        wrist.setPosition(wristScoringPos);
        armUp();
        wait(500);
        if(alliance == 1){
            String s = "RightCenter";
            if(result.equals("Right")){
                s = "CenterCenter";
            }
            positionOnBackdrop(s, alliance, 1);
            wait(500);
            positionOnBackdrop(s, alliance, 2);

        }else if(alliance == -1){
            String s = "LeftCenter";
            if(result.equals("Left")){
                s = "CenterCenter";
            }
            positionOnBackdrop(s, alliance, 1);
            wait(500);
            positionOnBackdrop(s, alliance, 2);
        }
        /*liftIdealPos = .07;
        while(Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()){
            liftWithinLoop();
        }*/
        openLowerClaw();
        RobotLog.aa("Status", "Opened first claw");
        wait(500);
        //sleep(200);
        liftIdealPos = liftSecondWhitePixelPos;
        RobotLog.aa("Status", "Set lift position");
        goStraight(.35, .5, -90.0*alliance);
        wait(500);
        goBackward(.35, .25, -90.0*alliance);
        RobotLog.aa("Status", "Moved and starting lift");
        /*liftPos = -((liftEncoder.getCurrentPosition()/ticksPerRotation)-liftInitial);
        while(Math.abs(liftIdealPos - liftPos) > .005 && opModeIsActive()){
            liftWithinLoop();
        }*/
        wait(500);
        RobotLog.aa("Status", "Lift moved up");
        openUpperClaw();
        RobotLog.aa("Status", "Claw opened");
        cameraBar.setPosition(camOutOfWay);
        wait(500);
    }
    public void park(int alliance, String result, boolean parkingNearWall){
        RobotLog.aa("Status", "Started parking");
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
                    strafeLeft(.6, 27, 5, -90.0*alliance);
                }else if(result.equals("Center")){
                    strafeLeft(.6, 22, 5, -90.0*alliance);
                }else{
                    strafeLeft(.6, 14, 5, -90.0*alliance);
                }
            }
        }
        sleep(1000);
    }
}
