package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.FastAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Autonomous(name = "Fast Red Right")
public class FastRedRight extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        //Vision
        int spikeLocation = robot.ScanForElementBitmap(2);
        sleep(1000);
        while (opModeInInit()) {
            spikeLocation = robot.ScanForElementBitmap(2);
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.addData("White One: ", robot.whiteOne);
            telemetry.addData("White Two: ", robot.whiteTwo);
            telemetry.addData("White Three: ", robot.whiteThree);
            telemetry.update();
        }

        waitForStart();
        robot.gameTimer.startTime();

        //Position 1
        if (spikeLocation == 1) {
            DriveWithCorrection(30, 0, 0.4);
            sleep(100);
            Turn(270);
            sleep(100);
            DriveWithCorrection(-2, 270, 0.2);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(5, 270, 0.3);
            sleep(100);
            Turn(90);
            sleep(100);
            DriveWithCorrection(-25, 90, 0.4);
            sleep(100);
            StrafeWithInchesWithCorrection(12, 0.3, 4, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 4);
        }

        //Position 2
        if (spikeLocation == 2) {
            DriveWithCorrection(18, 0, 0.3);
            sleep(100);
            DriveWithCorrection(27, 0, 0.4);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(2, 0, 0.2);
            sleep(100);
            Turn(90);
            DriveWithCorrection(-27, 90, 0.4);
            StrafeWithInchesWithCorrection(48, -0.3, 5, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 5);
        }

        //Position 3
        if (spikeLocation == 3) {
            DriveWithCorrection(30, 0, 0.4);
            sleep(100);
            Turn(90);
            sleep(100);
            DriveWithCorrection(-1.5, 90, 0.2);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            StrafeWithInchesWithCorrection(20, 0.3, -1, 90);
            sleep(100);
            DriveWithCorrection(-27, 90, 0.4);
            sleep(100);
            StrafeWithInchesWithCorrection(35, -0.3, 6, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 6);
        }

        lift.SetPosition(lift.liftLow - 4,  lift.liftAprilTags);
        sleep(100);
        DriveWithCorrection(-2, 90, 0.3);
        sleep(100);
        dropper.OpenDropper();
        sleep(400);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow, lift.liftLow - 5);
        sleep(100);

        Drive(3);
        sleep(100);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(100);
        StrafeWithInchesWithCorrection(12, 0.3, -1, 90);
        sleep(500);
    }

}
