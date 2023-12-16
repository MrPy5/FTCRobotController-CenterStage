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
        lift.SetPosition(lift.liftLow, 0);
        sleep(1000);
        dropper.OpenDropper();
        sleep(500);
        dropper.CloseDropper();
        sleep(500);
    }

}
