package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Right")
public class BlueRight extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

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
            Drive(15);
            sleep(1000);
            Drive(5);
            sleep(500);
            Turn(60);
            Drive(-2);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Turn(0);
            Drive(-23);
            Turn(90);
            Drive(89);
        }
        if (spikeLocation == 2) {
            Drive(17);
            sleep(1000);
            Drive(5);

            sleep(500);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Turn(90);
            Drive(84);

        }
        if (spikeLocation == 3) {
            Drive(18);
            sleep(1000);
            Drive(7);
            sleep(500);
            Turn(300);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Turn(90);
            Drive(84);

        }





    }

}
