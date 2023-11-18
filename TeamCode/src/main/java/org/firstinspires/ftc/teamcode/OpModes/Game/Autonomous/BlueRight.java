package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Right")
public class BlueRight extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

        int spikeLocation = robot.ScanForElement();
        sleep(5000);
        while (opModeInInit()) {

            spikeLocation = robot.ScanForElement();
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.update();
        }
        waitForStart();
        robot.gameTimer.startTime();

        if (spikeLocation == 1) {
            Drive(18);
            sleep(1000);
            Drive(6);
            sleep(500);
            Turn(90);
            Drive(-2);
            intake.RunToPosIntake(1000, 1);
            Turn(0);
            sleep(500);
            Drive(24);
            Turn(270);
            Drive(-60);
            StrafeWithInches(25, 1);
            Navigate(1, 4, 5);
            StrafeWithInches(2, 1);

        }
        if (spikeLocation == 2) {
            Drive(5);
            sleep(1000);
            Turn(270);
            sleep(500);
            Drive(10);
            sleep(500);
            Turn(0);
            sleep(500);
            Drive(45);
            sleep(500);
            Turn(270);
            sleep(500);
            Drive(-15);
            sleep(500);
            Turn(180);
            intake.RunToPosIntake(1000, 1);
            Turn(270);
            Drive(-60);
            StrafeWithInches(27, 1);
            Navigate(2, 4, 5);
            StrafeWithInches(2, 1);

        }
        if (spikeLocation == 3) {
            Drive(18);
            sleep(1000);
            Drive(6);
            sleep(500);
            Turn(270);
            Drive(-2);
            intake.RunToPosIntake(1000, 1);
            Turn(0);
            sleep(500);
            Drive(24);
            Turn(270);
            Drive(-60);
            StrafeWithInches(30, 1);
            Navigate(3, 4, 5);
            StrafeWithInches(2, 0);

        }


        Drive(-2);
        lift.SetPosition(lift.liftLow - 6, 0);
        sleep(2000);
        dropper.OpenDropper();
        sleep(1000);
        dropper.CloseDropper();
        Drive(3);
        sleep(1000);
        lift.SetPosition(lift.liftBottom, lift.liftLow - 6);
        sleep(4000);


    }

}
