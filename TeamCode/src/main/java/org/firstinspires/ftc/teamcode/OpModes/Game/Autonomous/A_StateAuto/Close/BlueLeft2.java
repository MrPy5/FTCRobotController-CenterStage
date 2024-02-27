package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.A_StateAuto.Close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;

@Autonomous(name = "Blue Left 2", group = "Close")
public class BlueLeft2 extends AutoControlsCombined {

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
        switchToContourPipeline();

        Motion driveOne = new Motion();
        if (spikeLocation == 1) {

            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 38, 0.5, 0));
            driveOne.add(new Strafe(new IndexTrigger(1, driveOne), 19, -0.5, 0, -1));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 4, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(5, driveOne), 3, 0.4, 270));
        }

        else if (spikeLocation == 2) {

            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Strafe(new MillisecondTrigger(0), 10, -0.5, 0, -1));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), 40, 0.5, 0));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(2, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 5, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(5, driveOne), -5, 0.4, 270));
        }

        else if (spikeLocation == 3) {

            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 21, 0.4, 0));
            driveOne.add(new Turn(new IndexTrigger(1, driveOne), 90));
            driveOne.add(new Drive(new IndexTrigger(2, driveOne), -1, 0.5, 90));
            driveOne.add(new SpikeDrop(new IndexTrigger(3, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(4, driveOne), 16, 0.25, 90));
            driveOne.add(new Turn(new IndexTrigger(6, driveOne), 270));

        }

        driveOne.Start(0);

        Motion driveTwo = new Motion();

        //Drive back toward backboard
        driveTwo.add(new Drive(new MillisecondTrigger(0), -13, 0.8, 270));
        driveTwo.add(new MoveLift(new IndexTrigger(0, driveTwo), lift.liftLow + 1, 0));

        driveTwo.Start(0);

        //Strafe on backboard
        if (spikeLocation == 1) {
            StrafeWithInchesWithCorrection(32, 0.15, 1, 270);
        }
        if (spikeLocation == 2) {
            StrafeWithInchesWithCorrection(26, 0.15, 2, 270);
        }
        if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(20, -0.15, 3, 270);
        }
        DriveWithCorrection(-16, 270, 0.4);

        dropper.OpenDropper();
        sleep(1000);
        lift.SetPosition(lift.liftLow + 7, 0, -1);
        sleep(1000);
        dropper.CloseDropper();
        DriveWithCorrection(4, 270, 0.4);
        lift.SetPosition(lift.liftBottom, lift.liftLow, -1);

        if (spikeLocation == 1) {
            StrafeWithInchesWithCorrection(16, 0.3, -1, 270);
        }
        if (spikeLocation == 2) {
            StrafeWithInchesWithCorrection(23, 0.3, -1, 270);
        }
        if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(29, 0.3, -1, 270);
        }

        robot.webcam.closeCameraDevice();
        robot.visionPortal.close();
    }

}
