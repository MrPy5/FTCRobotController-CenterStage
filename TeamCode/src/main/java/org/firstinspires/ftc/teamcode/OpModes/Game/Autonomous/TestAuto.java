package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test Auto")
public class TestAuto extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";


        waitForStart();
        robot.gameTimer.startTime();

        DriveWithCorrectionToAprilTag(-25, 0, 0.2, 5);

    }

}
