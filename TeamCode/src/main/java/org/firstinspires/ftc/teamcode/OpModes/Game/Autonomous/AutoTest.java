package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto")
public class AutoTest extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";
        waitForStart();
        robot.gameTimer.startTime();

        sleep(5000);
        telemetry.addData("center", robot.getCenter());
        telemetry.update();

        int spikeLocation = robot.ScanForElement();

        if (spikeLocation == 1) {
            Drive(18);
            sleep(1000);
            Drive(25);
            sleep(500);
            Turn(60);
            intake.StartIntake(-0.3);
            sleep(3000);
            intake.StopIntake();
            Turn(270);
        }
        if (spikeLocation == 2) {
            Drive(18);
            sleep(1000);
            Drive(25);

            sleep(500);
            intake.StartIntake(-0.3);
            sleep(3000);
            intake.StopIntake();

            Turn(270);

        }





    }

}
