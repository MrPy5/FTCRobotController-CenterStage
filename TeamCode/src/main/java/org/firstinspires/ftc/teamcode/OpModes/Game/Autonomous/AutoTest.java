package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto")
public class AutoTest extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        waitForStart();
        robot.gameTimer.startTime();
        int spikeLocation = robot.ScanForElement();

        if (spikeLocation == 2) {
            Drive(18);
            sleep(1000);
            Drive(25);

            sleep(500);
            intake.StartIntake(-0.3);
            sleep(3000);
            intake.StopIntake();

        }





    }

}
