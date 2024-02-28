package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.OldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.AutoControls;

@Disabled
@Autonomous(name = "Old Blue Left")
public class OldBlueLeft extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

        int spikeLocation = robot.ScanForElement(1);
        sleep(5000);
        while (opModeInInit()) {

            spikeLocation = robot.ScanForElement(1);
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
            StrafeWithInches(10, 0, 3);
            Navigate(3, 9, 5);
        }

        sleep(1000);
        lift.SetPosition(lift.liftLow - 4, lift.liftAprilTags, -1);
        sleep(500);
        Drive(-5);
        sleep(2000);
        dropper.OpenDropper();
        sleep(1000);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow, lift.liftLow - 5, -1);
        sleep(1000);
        Drive(3);
        sleep(1000);
        lift.SetPosition(lift.liftBottom, lift.liftLow, -1);
        sleep(4000);


    }

}
