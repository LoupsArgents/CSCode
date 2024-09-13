package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class TestTurningAuto extends PPBotCSDF {
    //centered lineup: right edge of driverail is on the left edge of the third tab (including the little half-tab on the very edge) from the right of the tile
    //so there are 2 indents to the right of it
    public void runOpMode() {
        initializeHardware();
        waitForStart();
        while (opModeIsActive()) {
            absoluteHeading(.25, -90.0);
            //strafeLeft(.35, 4, 5, -180.0);
        }
    }
}
