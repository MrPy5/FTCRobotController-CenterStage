package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Autonomous(name = "Red Right")
public class RedRight extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        //Vision
        /*
        int spikeLocation = robot.ScanForElement(1);
        sleep(1000);
        while (opModeInInit()) {
            spikeLocation = robot.ScanForElement(1);
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.addData("Area: ", robot.area);
            telemetry.update();
        }
         */

        int spikeLocation = robot.ScanForElementBitmap(2);
        sleep(1000);
        while (opModeInInit()) {
            spikeLocation = robot.ScanForElementBitmap(2);
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.addData("White One: ", robot.whiteOne);
            telemetry.addData("White Two: ", robot.whiteTwo);
            telemetry.update();
        }

        waitForStart();
        robot.gameTimer.startTime();

        //Position 1
        if (spikeLocation == 1) {
            DriveWithCorrection(30, 0, 0.2);
            sleep(500);
            Turn(270);
            sleep(500);
            DriveWithCorrection(-2, 270, 0.2);
            sleep(500);
            spike.DropSpike();
            sleep(500);
            DriveWithCorrection(3, 270, 0.2);
            sleep(500);
            Turn(90);
            sleep(500);
            DriveWithCorrection(-28, 90, 0.2);
            sleep(500);
            StrafeWithInches(12, 1, 4);
            //Navigate(4, 9, 5);
        }
        //Position 2
        if (spikeLocation == 2) {
            DriveWithCorrection(47, 0, 0.2);
            sleep(500);
            spike.DropSpike();
            sleep(1000);
            Turn(90);
            DriveWithCorrection(-30, 90, 0.2);
            StrafeWithInches(48, 0, 5);
            //Navigate(5, 9, 5);
        }
        //Position 3
        if (spikeLocation == 3) {
            DriveWithCorrection(30, 0, 0.2);
            sleep(500);
            Turn(90);
            sleep(500);
            DriveWithCorrection(-3.5, 90, 0.2);
            sleep(500);
            spike.DropSpike();
            sleep(500);
            StrafeWithInches(20, 1, -1);
            sleep(500);
            DriveWithCorrection(-27, 90, 0.2);
            sleep(500);
            StrafeWithInches(35, 0, 6);
            //Navigate(6, 9, 5);
        }

        DriveWithCorrection(-5, 90, 0.3);
        sleep(1000);
        lift.SetPosition(lift.liftLow - 4,  lift.liftAprilTags);
        sleep(500);
        Drive(-5);
        sleep(1000);
        dropper.OpenDropper();
        sleep(500);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow, lift.liftLow - 5);
        sleep(1000);

        Drive(3);
        sleep(1000);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(1000);

    }

}
