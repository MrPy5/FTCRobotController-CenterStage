package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test Auto")
public class TestAuto extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        waitForStart();
        robot.gameTimer.startTime();

        lift.SetPosition(lift.liftLow, 0);
        Navigate(6, 6, 5);






    }

}
