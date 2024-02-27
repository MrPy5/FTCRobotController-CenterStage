package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.AutoTwoPixels;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;

@Autonomous(name = "Fast Red Right", group = "Fast")
public class FastRedRight extends AutoControlsCombined {

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
        switchToContourPipeline();

        Motion driveOne = new Motion();
        if (spikeLocation == 1) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 28, 0.4, 0));
            driveOne.add(new Turn(new IndexTrigger(1, driveOne), 270));
            driveOne.add(new Drive(new IndexTrigger(2, driveOne), -5, 0.5, 270));
            driveOne.add(new SpikeDrop(new IndexTrigger(3, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(4, driveOne), 11, 0.25, 270));
            driveOne.add(new Turn(new IndexTrigger(6, driveOne), 90));
        }

        else if (spikeLocation == 2) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Strafe(new MillisecondTrigger(0), 11, 0.5, 0, -1));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), 40, 0.5, 0));
            driveOne.add(new Strafe(new IndexTrigger(2, driveOne), 15, -0.5, 0, -1));
            driveOne.add(new SpikeDrop(new IndexTrigger(3, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(4, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(4, driveOne), 4, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(6, driveOne), -10, 0.4, 90));


        }

        else if (spikeLocation == 3) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 38, 0.5, 0));
            driveOne.add(new Strafe(new IndexTrigger(1, driveOne), 4, 0.5, 0, -1));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 4, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(5, driveOne), -8, 0.4, 90));
        }

        driveOne.Start(0);

        Motion driveTwo = new Motion();

        //Drive back toward backboard
        driveTwo.add(new Drive(new MillisecondTrigger(0), -13, 0.8, 90));
        driveTwo.add(new MoveLift(new IndexTrigger(0, driveTwo), lift.liftLow + 1, 0));

        driveTwo.Start(0);

        //Strafe on backboard
        if (spikeLocation == 1) {
            StrafeWithInchesWithCorrection(15, 0.15, 4, 90);
        }
        if (spikeLocation == 2) {
            StrafeWithInchesWithCorrection(26, -0.15, 5, 90);
        }
        if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(32, -0.15, 6, 90);
        }
        DriveWithCorrection(-16, 90, 0.4);

        dropper.OpenDropper();
        sleep(1000);
        lift.SetPosition(lift.liftLow + 7, 0, -1);
        sleep(1000);
        dropper.CloseDropper();
        DriveWithCorrection(4, 90, 0.4);
        lift.SetPosition(lift.liftBottom, lift.liftLow, -1);
        if (spikeLocation == 1) {
            StrafeWithInchesWithCorrection(29, -0.3, -1, 90);
        }
        if (spikeLocation == 2) {
            StrafeWithInchesWithCorrection(26, -0.3, -1, 90);
        }
        if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(16, -0.3, -1, 90);
        }

    }

}
