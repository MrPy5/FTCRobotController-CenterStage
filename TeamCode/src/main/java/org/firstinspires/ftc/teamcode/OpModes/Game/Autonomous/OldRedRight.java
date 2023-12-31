package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Old Red Right")
public class OldRedRight extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        int spikeLocation = robot.ScanForElement(1);
        sleep(5000);
        while (opModeInInit()) {

            spikeLocation = robot.ScanForElement(1);
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.addData("Area: ", robot.area);
            telemetry.update();
        }
        waitForStart();
        robot.gameTimer.startTime();
        double drive = 0;
        if (spikeLocation == 1) {
            Drive(18);
            sleep(1000);
            Drive(6);
            sleep(500);
            Turn(90);
            Drive(-4);
            intake.RunToPosIntake(-1000, 1);
            Drive(-15);
            StrafeWithInches(5, 1, 4);
            Navigate(4, 9, 5);


        }
        if (spikeLocation == 2) {
            Drive(18);
            sleep(1000);
            Drive(3);

            sleep(500);
            intake.RunToPosIntake(-1000, 1);
            Drive(-2);
            Turn(90);
            Drive(-15);
            drive = StrafeWithInches(5, 1, 5);
            //Navigate(5, 9, 5);

        }
        if (spikeLocation == 3) {
            Turn(270);
            Drive(24);
            sleep(500);
            Turn(0);
            sleep(500);
            Drive(26);
            sleep(500);
            Turn(90);
            sleep(250);
            Drive(-2);
            intake.RunToPosIntake(-1000,1);
            drive = StrafeWithInches(7, 0, 6);
            //Navigate(6, 9, 5);

        }

        Drive(-drive);
        sleep(1000);
        lift.SetPosition(lift.liftLow - 4,  lift.liftAprilTags);
        sleep(500);
        Drive(-5);
        sleep(2000);
        dropper.OpenDropper();
        sleep(1000);
        dropper.CloseDropper();
        sleep(500);
        lift.SetPosition(lift.liftLow, lift.liftLow - 5);
        sleep(1000);

        Drive(3);
        sleep(1000);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(1000);
        StrafeWithInches(26, 0, -1);


    }

}
