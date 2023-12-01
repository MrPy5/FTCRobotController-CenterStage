package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Left")
public class BlueLeft extends FirstMeetAutoControls {

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
            Turn(90);
            Drive(22);
            sleep(500);
            Turn(0);
            sleep(500);
            Drive(26);
            sleep(500);
            Turn(270);
            sleep(250);
            Drive(1);
            intake.RunToPosIntake(-1000,1);
            StrafeWithInches(9, 1, 1);
            Navigate(1, 9, 5);


        }
        if (spikeLocation == 2) {
            Drive(18);
            sleep(1000);
            Drive(3);

            sleep(500);
            intake.RunToPosIntake(-1000, 1);
            Turn(270);
            //Navigate(2, 8, 5);
            Drive(-15);
            Navigate(2, 9, 5);
        }
        if (spikeLocation == 3) {

            Drive(18);
            sleep(1000);
            Drive(6);
            sleep(500);
            Turn(270);
            Drive(-1);
            intake.RunToPosIntake(-1000, 1);
            Drive(-10);
            Navigate(3, 9, 5);
        }

        sleep(1000);
        Drive(-5);
        lift.SetPosition(lift.liftLow - 5, 0);
        sleep(2000);
        dropper.OpenDropper();
        sleep(1000);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow, lift.liftLow - 5);
        sleep(1000);
        Drive(3);
        sleep(1000);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(4000);


    }

}
