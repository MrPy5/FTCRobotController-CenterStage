package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.A_StateAuto.Far.TwoThree;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;

@Autonomous(name = "Red Left 2+3", group = "2+3")
public class RedLeft2_3 extends AutoControlsCombined {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        double previousStrafe = 0;


        //Vision
        int spikeLocation = robot.ScanForElementBitmap(2);
        sleep(1000);
        while (opModeInInit()) {
            spikeLocation = robot.ScanForElementBitmap(2);
            telemetry.addData("Spikes: ", spikeLocation);
            telemetry.addData("White One: ", robot.whiteOne);
            telemetry.addData("White Two: ", robot.whiteTwo);
            telemetry.addData("White Three: ", robot.whiteThree);
            telemetry.addData("fps: ", robot.visionPortal.getFps());


            if (robot.visionPortal.getFps() == 0.0) {
                sleep(1000);
                robot.visionPortal.close();
                sleep(1000);
                robot.initAprilTag();
                telemetry.addData("reset", "Reset");
                telemetry.update();
                sleep(1000);
            }
            telemetry.update();
        }

        //waitForStart();
        switchToContourPipeline();

        Motion driveOne = new Motion();
        if (spikeLocation == 1) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));

            //Drive to spike drop position, drop purple pixel
            driveOne.add(new Drive(new MillisecondTrigger(0), 37.5, 0.5, 0));
            driveOne.add(new Strafe(new IndexTrigger(1, driveOne), 18, -0.4, 0, -1));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));

            driveOne.add(new Drive(new IndexTrigger(4, driveOne), 8, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(5, driveOne), -10, 0.4, 90));
        }

        else if (spikeLocation == 2) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new CatWalk(new MillisecondTrigger(0), 40.5, -17, 0.5, 0, 0.5, new Vision(), 0.5));
            driveOne.add(new SpikeDrop(new IndexTrigger(1, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(2, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(2, driveOne), 8, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(4, driveOne), -3.5, 0.4, 90));


        }

        else if (spikeLocation == 3) {
            //Initial Spike drop
            //Take first pixel from stack
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 26.25, 0.5, 0));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), -1, 0.4, 90));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 10, 0.25, 90));
            //driveOne.add(new CatWalk(new IndexTrigger(5, driveOne), 2, 20.75, 0.4, 90, 0.75, new Vision(), 0.5));
            driveOne.add(new Strafe(new IndexTrigger(5, driveOne), 18.75, 0.4, 90, -1));

            //driveOne.add(new MoveHoist(new IndexTrigger(6, driveOne), hoist.stackPosition5));
            //driveOne.add(new MoveIntake(new IndexTrigger(6, driveOne), 1500));
        }
        driveOne.Start(0);
        if (spikeLocation == 1) {
            DriveAtAngleToStack(18, 0.3);
        }
        if (spikeLocation == 2) {
            DriveAtAngleToStack(22.5, 0.3);
        }
        if (spikeLocation == 3) {
            DriveAtAngleToStack(19.5, 0.3);
        }



       Motion driveTwo = new Motion();

            driveTwo.add(new MoveHoist(new MillisecondTrigger(0), hoist.stackPosition5));
            driveTwo.add(new MoveIntake(new MillisecondTrigger(0), 1000));

            //Drive back toward backboard
            driveTwo.add(new MoveIntake(new IndexTrigger(1, driveTwo), 2000));
            driveTwo.add(new Drive(new IndexTrigger(1, driveTwo), -100, 1, 90));
            driveTwo.add(new MoveLift(new IndexTrigger(3, driveTwo), lift.liftLow - 1, 0));
            //driveTwo.add(new CatWalk(new IndexTrigger(1, driveTwo), -28, -28, 0.5, 90, 0.5, new Vision()));

        driveTwo.Start(0);
        hoist.Hoist();
       //Strafe on backboard - based on spike position
       if (spikeLocation == 1) {
           previousStrafe = StrafeWithInchesWithCorrectionWithDistanceSensors(21, -0.25, 4, 90);
       }
       if (spikeLocation == 2) {
            previousStrafe = StrafeWithInchesWithCorrectionWithDistanceSensors(26, -0.25, 5, 90);
       }
       if (spikeLocation == 3) {
            previousStrafe = StrafeWithInchesWithCorrectionWithDistanceSensors(36, -0.25, 6, 90);
       }

       //Drive back to backboard
       DriveWithCorrection(-10, 90, 0.3);

       Motion driveThree = new Motion();
            //Drop pixel, raise lift, drive forward
            driveThree.add(new MoveDropper(new MillisecondTrigger(0), "open"));
            driveThree.add(new MoveLift(new IndexTriggerWithDelay(0, 300, driveThree), lift.liftLow + 5, lift.liftLow - 1));
            driveThree.add(new Drive(new IndexTriggerWithDelay(1, 300, driveThree), 5, 0.5, 90));
            driveThree.add(new MoveLift(new IndexTriggerWithDelay(2, 0, driveThree), lift.liftBottom, lift.liftLow + 4));
            driveThree.add(new MoveDropper(new IndexTriggerWithDelay(2, 0, driveThree), "close"));
            driveThree.add(new MoveHoist(new IndexTrigger(3, driveThree), hoist.stackPosition3));



        driveThree.Start(0);


        if (spikeLocation == 1) {
            //driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, 18, 0.5, 90, 0.5, new Vision(), 1));
            StrafeWithInchesWithCorrection(17.5, 0.5, -1, 90);

            DriveWithCorrection(80, 90, 0.9);
        }
        if (spikeLocation == 2) {
            //driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, 23, 0.5, 90, 0.5, new Vision(), 1));
            StrafeWithInchesWithCorrection(26, 0.5, -1, 90);
            //22.5 ORIGINALLY
            DriveWithCorrection(80, 90, 0.9);
        }
        if (spikeLocation == 3) {
            //driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, 29, 0.5, 90, 0.5, new Vision(), 1));
            StrafeWithInchesWithCorrection(29, 0.5, -1, 90);

            DriveWithCorrection(80, 90, 0.9);
        }


        /*
        StrafeWithInchesWithCorrection(previousStrafe - 4, 0.8, -1, 90);
        DriveWithCorrection(76, 90, 1);
         */

        //Align to stack

        DriveAtAngleToStack(24.5, .2);

        Motion driveFour = new Motion();
            //Intake pixels and move to backboard
            driveFour.add(new MoveIntake(new MillisecondTrigger(0), 1800));
            driveFour.add(new MoveHoist(new IndexTrigger(0, driveFour), hoist.stackPosition3));
            driveFour.add(new MoveIntake(new IndexTrigger(1, driveFour), 1800));
            driveFour.add(new Drive(new MillisecondTrigger(1500), -100, 0.9, 90));

            driveFour.add(new MoveLift(new IndexTrigger(3, driveFour), lift.liftLow + 2, 0));




        driveFour.Start(0);
        //Strafe along board
        if (spikeLocation == 1) {
            StrafeWithInchesWithCorrectionWithDistanceSensors(33, -0.3, 5, 90);
        } else {
            StrafeWithInchesWithCorrectionWithDistanceSensors(21, -0.3, 4, 90);
        }


        DriveWithCorrection(-10, 90, 0.3);
        dropper.OpenDropper();
        sleep(500);
        lift.SetPosition(lift.liftLow + 6, lift.liftLow, -1);
        sleep(600);
    }

}
