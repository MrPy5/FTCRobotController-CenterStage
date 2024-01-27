package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.FastAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;
@Disabled
@Autonomous(name = "Fast Blue Right", group = "Fast Autos")
public class FastBlueRight extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

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
            DriveWithCorrection(39, 0, 0.5);
            sleep(100);
            Turn(270);
            sleep(100);
            DriveWithCorrection(-2, 270, 0.2);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(3, 270, 0.2);
            sleep(100);
            StrafeWithInchesWithCorrection(11, -0.3, -1, 270);
            sleep(100);
            DriveWithCorrection(-75, 270, 0.6);
            sleep(100);
            StrafeWithInchesWithCorrection(48, 0.3, 1, 270);
            DriveWithCorrectionToAprilTag(-20, 270, 0.5, 1);
        }

        //Position 2
        if (spikeLocation == 2) {
            /*
            DriveWithCorrection(18, 0, 0.2);
            sleep(500);
            DriveWithCorrection(27, 0, 0.2);
            sleep(500);
            spike.DropSpike();
            sleep(1000);
            spike.ResetSpike();
            DriveWithCorrection(2, 0, 0.2);
            sleep(250);
            Turn(270);
            DriveWithCorrection(-72, 270, 0.2);
            StrafeWithInchesWithCorrection(48, 0.2, 2, 270);
            DriveWithCorrectionToAprilTag(-20, 270, 0.4, 2);*/
            DriveWithCorrection(18, 0, 0.3);
            sleep(100);
            StrafeWithInchesWithCorrection(10, 0.3, -1, 0);
            sleep(100);
            DriveWithCorrection(26.5, 0, 0.4);
            sleep(100);
            StrafeWithInchesWithCorrection(12, -0.3, -1, 0);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(3, 0, 0.2);
            sleep(100);
            Turn(270);
            DriveWithCorrection(-72, 270, 0.6);
            StrafeWithInchesWithCorrection(48, 0.3, 2, 270);
            DriveWithCorrectionToAprilTag(-20, 270, 0.4, 2);

        }

        //Position 3
        if (spikeLocation == 3) {
            DriveWithCorrection(42, 0, 0.4);
            sleep(100);
            StrafeWithInchesWithCorrection(5, 0.3, -1, 0);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(4, 0, 0.3);
            Turn(270);
            DriveWithCorrection(-78, 270, 0.6);
            sleep(100);
            StrafeWithInchesWithCorrection(48, 0.3, 3, 270);
            DriveWithCorrectionToAprilTag(-40, 270, 0.4, 3);
        }

        lift.SetPosition(lift.liftLow - 4,  lift.liftAprilTags);
        sleep(200);
        DriveWithCorrection(-2, 270, 0.3);
        sleep(50);
        dropper.OpenDropper();
        sleep(400);
        dropper.CloseDropper();
        sleep(50);
        lift.SetPosition(lift.liftLow, lift.liftLow - 5);
        sleep(100);

        Drive(3);
        sleep(200);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(500);

        if (spikeLocation == 1) {
            StrafeWithInchesWithCorrection(28, -0.4, -1, 270);
        } else if (spikeLocation == 2) {
            StrafeWithInchesWithCorrection(22, -0.4, -1, 270);
        } else if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(18, -0.4, -1, 270);
        }

        DriveWithCorrection(78, 270, 0.6);
        DriveWithCorrectionToStack(27, 270, 0.3);
        hoist.Stack();
        DriveWithCorrection(4, 270, 0.2);
        intake.StartIntake(1);
        sleep(1750);
        intake.StopIntake();
        DriveWithCorrection(-4, 270, 0.2);
        hoist.Ground();

        intake.StartIntake(0.6);
        DriveWithCorrection(-83, 270, 0.6);
        intake.StopIntake();

        StrafeWithInchesWithCorrection(48, -0.3, 4, 270);
        DriveWithCorrectionToAprilTag(-55, 270, 0.4, 4);

        lift.SetPosition(lift.liftLow - 1,  lift.liftAprilTags);
        sleep(300);
        DriveWithCorrection(-2, 270, 0.3);
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
