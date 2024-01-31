package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.OldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Disabled
@Autonomous(name = "Old Blue Right")
public class OldBlueRight extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

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

        if (spikeLocation == 1) {
            Drive(18);
            sleep(1000);
            Drive(6);
            sleep(500);
            Turn(90);
            Drive(-4);
            intake.RunToPosIntake(-1000, 1);
            Turn(0);
            sleep(500);
            Drive(24);
            Turn(270);
            Drive(-80);
            Turn(270);
            StrafeWithInches(30, 1, 1);
            Navigate(1, 9, 5);

        }
        if (spikeLocation == 2) {
            Drive(5);
            sleep(1000);
            Turn(160);
            sleep(500);
            Drive(-43);
            sleep(500);
            Turn(180);
            StrafeWithInches(12, 1, -1);
            intake.RunToPosIntake(-1000, 1);
            sleep(500);
            Drive(-3);
            Turn(270);
            Drive(-76);
            Turn(270);
            StrafeWithInches(30, 1, 2);
            Navigate(2, 9, 5);

        }
        if (spikeLocation == 3) {
            Drive(18);
            sleep(1000);
            Drive(6);
            sleep(500);
            Turn(270);
            Drive(-3);
            intake.RunToPosIntake(-1000, 1);
            Drive(-1);
            Turn(0);
            sleep(500);
            Drive(24);
            Turn(270);
            Drive(-79);
            Turn(270);
            StrafeWithInches(20, 1, 3);
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
