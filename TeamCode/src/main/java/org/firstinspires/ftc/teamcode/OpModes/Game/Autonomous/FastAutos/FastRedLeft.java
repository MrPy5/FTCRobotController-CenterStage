package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.FastAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Autonomous(name = "Fast Red Left")
public class FastRedLeft extends AutoControls {

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

        //int spikeLocation = 1;
        waitForStart();
        switchToContourPipeline();

        robot.gameTimer.startTime();

        //Position 1
        if (spikeLocation == 1) {
            DriveWithCorrection(42, 0, 0.4);
            sleep(100);
            StrafeWithInchesWithCorrection(14, -0.4, -1, 0);
            sleep(100);
            DriveWithCorrection(-1, 0, 0.3);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(5, 0, 0.4);
            Turn(90);
            DriveWithCorrection(-88, 90, 0.6);
            sleep(100);
            StrafeWithInchesWithCorrection(48, -0.3, 4, 90);
            DriveWithCorrectionToAprilTag(-40, 90, 0.4, 4);

        }

        //Position 2
        if (spikeLocation == 2) {
            DriveWithCorrection(18, 0, 0.4);
            sleep(100);
            StrafeWithInchesWithCorrection(10, -0.3, -1, 0);
            sleep(100);
            DriveWithCorrection(24.5, 0, 0.5);
            sleep(100);
            StrafeWithInchesWithCorrection(2, 0.3, -1, 0);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(4, 0, 0.3);
            sleep(100);
            Turn(90);
            DriveWithCorrection(-82, 90, 0.6);
            StrafeWithInchesWithCorrection(48, -0.3, 5, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 5);
        }

        //Position 3
        if (spikeLocation == 3) {
            DriveWithCorrection(28, 0, 0.4);
            sleep(100);
            Turn(90);
            sleep(100);
            DriveWithCorrection(-3, 90, 0.3);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(2, 90, 0.3);
            sleep(100);
            StrafeWithInchesWithCorrection(20, 0.3, -1, 90);
            sleep(100);
            DriveWithCorrection(-75, 90, 0.6);
            sleep(100);
            StrafeWithInchesWithCorrection(48, -0.3, 6, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 6);
        }

        lift.SetPosition(lift.liftLow - 5,  lift.liftAprilTags);
        sleep(200);
        DriveWithCorrection(-2, 90, 0.3);
        sleep(50);
        dropper.OpenDropper();
        sleep(600);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow, lift.liftLow - 5);
        sleep(50);

        Drive(3);
        sleep(200);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(500);

        if (spikeLocation == 1) {
            StrafeWithInchesWithCorrection(18, 0.4, -1, 90);
        } else if (spikeLocation == 2) {
            StrafeWithInchesWithCorrection(22, 0.4, -1, 90);
        } else if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(28, 0.4, -1, 90);
        }

        DriveWithCorrection(78, 90, 0.6);
        DriveWithCorrectionToStack(27, 90, 0.3);
        hoist.Stack();
        DriveWithCorrection(4, 90, 0.2);
        intake.StartIntake(1);
        sleep(1750);
        intake.StopIntake();
        DriveWithCorrection(-4, 90, 0.2);
        hoist.Ground();

        intake.StartIntake(0.6);
        DriveWithCorrection(-83, 90, 0.6);
        intake.StopIntake();

        StrafeWithInchesWithCorrection(48, -0.3, 4, 90);
        DriveWithCorrectionToAprilTag(-55, 90, 0.4, 4);

        lift.SetPosition(lift.liftLow - 1,  lift.liftAprilTags);
        sleep(300);
        DriveWithCorrection(-2, 90, 0.3);
        sleep(50);
        dropper.OpenDropper();
        sleep(600);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow + 2, lift.liftLow - 5);
        sleep(50);

        Drive(3);
        sleep(200);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(500);
    }

}
