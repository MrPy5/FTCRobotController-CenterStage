package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAutoTWO")
public class TestAutoTwo extends AutoControlsCombined {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";
        switchToContourPipeline();
        waitForStart();
        hoist.Stack();
        DriveWithCorrectionToStackTwo(24, 0, 0.2);
        intake.StartIntake(0.75);

        sleep(3000);
        intake.StopIntake();




    }

}
