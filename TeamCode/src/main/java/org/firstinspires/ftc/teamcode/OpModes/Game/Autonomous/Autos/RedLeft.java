package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Autonomous(name = "Red Left")
public class RedLeft extends AutoControls {

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
            DriveWithCorrection(1, 270, 0.2);
            sleep(500);
            StrafeWithInches(24, 0, -1);
            sleep(500);
            Turn(90);
            sleep(500);
            DriveWithCorrection(-72, 90, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(48, 0, 4, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.2, 4);
        }
        //Position 2
        if (spikeLocation == 2) {
            DriveWithCorrection(47, 0, 0.2);
            sleep(500);
            spike.DropSpike();
            sleep(1000);
            Turn(90);
            DriveWithCorrection(-72, 90, 0.2);
            StrafeWithInchesWithCorrection(48, 0, 5, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.2, 5);
        }
        //Position 3
        if (spikeLocation == 3) {
            DriveWithCorrection(30, 0, 0.2);
            sleep(500);
            Turn(90);
            sleep(500);
            DriveWithCorrection(-3, 90, 0.2);
            spike.DropSpike();
            sleep(500);
            DriveWithCorrection(2, 90, 0.2);
            sleep(500);
            StrafeWithInches(22, 1, -1);
            sleep(500);
            DriveWithCorrection(-70, 90, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(48, 0, 6, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.2, 6);
        }

        Drive(-2);
        sleep(1000);
        lift.SetPosition(lift.liftLow - 4,  lift.liftAprilTags);
        sleep(500);
        Drive(-7);
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
