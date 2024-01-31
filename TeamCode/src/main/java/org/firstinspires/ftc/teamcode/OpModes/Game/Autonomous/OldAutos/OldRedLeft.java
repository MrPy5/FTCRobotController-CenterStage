package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.OldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Disabled
@Autonomous(name = "Old Red Left")
public class OldRedLeft extends AutoControls {

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
        sleep(5000);
        double drive = 0;
        if (spikeLocation == 1) {
            Drive(18);
            sleep(1000);
            Drive(6);
            sleep(500);
            Turn(90);
            Drive(-2);
            intake.RunToPosIntake(-1000, 1);
            Turn(0);
            sleep(500);
            Drive(24);
            Turn(90);
            Turn(90);
            Drive(-64);
            drive = StrafeWithInches(30, 0, 4);
            //Navigate(4, 9, 5);
        }
        if (spikeLocation == 2) {
            Drive(5);
            sleep(1000);
            Turn(90);
            sleep(500);
            Drive(10);
            sleep(500);
            Turn(0);
            sleep(500);
            Drive(45);
            sleep(500);
            Turn(90);
            sleep(500);
            Drive(-12);
            sleep(500);
            Turn(180);
            intake.RunToPosIntake(-1000, 1);
            Turn(90);
            Drive(-70);
            Turn(90);
            drive = StrafeWithInches(30, 0, 5);
            //Navigate(5, 9, 5);
        }
        if (spikeLocation == 3) {
            Drive(18);
            sleep(1000);
            Drive(6);
            sleep(500);
            Turn(270);
            Drive(-2);
            intake.RunToPosIntake(-1000, 1);
            Drive(-1);
            Turn(0);
            sleep(500);
            Drive(22);
            Turn(90);
            Drive(-75);
            Turn(90);
            drive = StrafeWithInches(30, 0, 6);
            //Navigate(6, 9, 5);
        }
        Drive(-13);
        sleep(1000);
        lift.SetPosition(lift.liftLow - 4,  lift.liftAprilTags, -1);
        sleep(500);
        Drive(-5);
        sleep(1000);
        dropper.OpenDropper();
        sleep(500);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow, lift.liftLow - 5, -1);
        sleep(1000);

        Drive(3);
        sleep(1000);
        lift.SetPosition(lift.liftBottom, lift.liftLow, -1);
        sleep(1000);


    }

}
