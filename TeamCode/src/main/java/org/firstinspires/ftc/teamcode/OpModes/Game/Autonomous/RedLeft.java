package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Left")
public class RedLeft extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        int spikeLocation = robot.ScanForElement();
        while (opModeInInit()) {

            spikeLocation = robot.ScanForElement();
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.update();
        }
        waitForStart();
        robot.gameTimer.startTime();

        telemetry.addData("center", robot.getCenter());
        telemetry.update();


        if (spikeLocation == 1) {
            Drive(18);
            sleep(1000);
            Drive(7);
            sleep(500);
            Turn(60);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Turn(270);
            Drive(84);
        }
        if (spikeLocation == 2) {
            Drive(15);
            sleep(1000);
            Drive(5);

            sleep(500);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Turn(270);
            Drive(84);

        }
        if (spikeLocation == 3) {
            Drive(18);
            sleep(1000);
            Drive(7);
            sleep(500);
            Turn(300);
            Drive(1);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Turn(1);
            Drive(-23);
            Turn(270);
            Drive(89);
        }





    }

}
