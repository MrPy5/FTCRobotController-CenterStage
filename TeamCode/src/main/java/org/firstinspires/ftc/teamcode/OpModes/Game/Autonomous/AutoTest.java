package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto")
public class AutoTest extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {

        initMethods(hardwareMap);
        waitForStart();
        robot.gameTimer.startTime();
        robot.frontLeft.setPower(1);
        sleep(5000);
        /*int spikeLocation = 2;//robot.ScanForElement();

        if (spikeLocation == 2) {
            Drive(30);
            sleep(500);
            Drive(24);
            sleep(500);
            intake.StartIntake(0.5);
            sleep(1);
            intake.StopIntake();
        }*/





    }

}
