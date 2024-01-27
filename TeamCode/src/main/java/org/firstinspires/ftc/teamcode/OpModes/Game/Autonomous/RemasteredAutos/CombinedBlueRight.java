package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.RemasteredAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;

@Autonomous(name = "Combined Blue Right", group = "Remastered")
public class CombinedBlueRight extends AutoControlsCombined {

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
            //Initial Spike drop
            //Take first pixel from stack
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 27.5, 0.5, 0));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), -4, 0.4, 270));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition5));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 7, 0.25, 270));
            //driveOne.add(new CatWalk(new IndexTrigger(5, driveOne), 23, 20.5, 0.4, 90, 1.2, new Vision()));
            driveOne.add(new CatWalk(new IndexTrigger(5, driveOne), 0, -19.5, 0.4, 270, 1.2, new Vision(), 0.5));

            //driveOne.add(new MoveHoist(new IndexTrigger(6, driveOne), hoist.stackPosition5));
            //driveOne.add(new MoveIntake(new IndexTrigger(6, driveOne), 1500));
        }

        else if (spikeLocation == 2) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new CatWalk(new MillisecondTrigger(0), 35, 5, 0.5, 0, 0.5, new Vision(), 0.5));
            driveOne.add(new CatWalk(new IndexTrigger(1, driveOne), 5, -19, 0.3, 0, 0.5, new Vision(), 1));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition5));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 6, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(5, driveOne), 4, 0.4, 270));


        }

        else if (spikeLocation == 3) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new CatWalk(new MillisecondTrigger(0), 40, 7, 0.5, 0, 0.5, new Vision(), 0.5));
            driveOne.add(new SpikeDrop(new IndexTrigger(1, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(1, driveOne), hoist.stackPosition5));
            driveOne.add(new Drive(new IndexTrigger(2, driveOne), 4, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(4, driveOne), -8, 0.4, 270));
        }
        driveOne.Start(0);

        if (spikeLocation == 1) {
            DriveAtAngleToStack(18, 0.3);
        }
        if (spikeLocation == 2) {
            DriveAtAngleToStack(21.5, 0.3);
        }
        if (spikeLocation == 3) {
            DriveAtAngleToStack(22.5, 0.3);
        }



       Motion driveTwo = new Motion();

            driveTwo.add(new MoveHoist(new MillisecondTrigger(0), hoist.stackPosition5));
            driveTwo.add(new MoveIntake(new MillisecondTrigger(0), 1500));

            //Drive back toward backboard
            driveTwo.add(new MoveIntake(new IndexTrigger(1, driveTwo), 2000));
            driveTwo.add(new Drive(new IndexTrigger(1, driveTwo), -96, 0.8, 270));
            driveTwo.add(new MoveLift(new IndexTrigger(3, driveTwo), lift.liftLow - 4, 0));
            //driveTwo.add(new CatWalk(new IndexTrigger(1, driveTwo), -28, -28, 0.5, 90, 0.5, new Vision()));

        driveTwo.Start(0);

       //Strafe on backboard
       if (spikeLocation == 1) {
           StrafeWithInchesWithCorrection(15, 0.25, 1, 270);
       }
       if (spikeLocation == 2) {
            StrafeWithInchesWithCorrection(20, 0.25, 2, 270);
       }
       if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(34, 0.25, 3, 270);
       }
       DriveWithCorrection(-10.5, 270, 0.4);

       Motion driveThree = new Motion();
            //Drop pixel and move back to stack
            driveThree.add(new MoveDropper(new MillisecondTrigger(0), "open"));
            driveThree.add(new MoveLift(new IndexTriggerWithDelay(0, 1500, driveThree), lift.liftBottom, lift.liftLow - 3));
            driveThree.add(new Drive(new IndexTriggerWithDelay(0, 1000, driveThree), 10, 0.5, 270));
            driveThree.add(new MoveDropper(new IndexTriggerWithDelay(0, 2000, driveThree), "close"));
            if (spikeLocation == 1) {
                driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, -29, 0.5, 270, 0.5, new Vision(), 1));

            }
            if (spikeLocation == 2) {
                driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, -26, 0.5, 270, 0.5, new Vision(), 1));

            }
            if (spikeLocation == 3) {
                driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, -14, 0.5, 270, 0.5, new Vision(), 1));

            }
            driveThree.add(new MoveHoist(new IndexTrigger(2, driveThree), hoist.stackPosition3));


        driveThree.Start(0);

        //Align to stack
        DriveAtAngleToStack(15.5, .2);

        Motion driveFour = new Motion();
            //Intake pixels and move to backboard
            driveFour.add(new MoveIntake(new MillisecondTrigger(0), 1800));
            driveFour.add(new MoveHoist(new IndexTrigger(0, driveFour), hoist.stackPosition3));
            driveFour.add(new MoveIntake(new IndexTrigger(1, driveFour), 1800));
            driveFour.add(new Drive(new MillisecondTrigger(1500), -100, 1, 270));
            driveFour.add(new MoveLift(new IndexTrigger(3, driveFour), lift.liftLow - 4, 0));

        driveFour.Start(0);
        //Strafe along board
        StrafeWithInchesWithCorrection(29, 0.25, 3, 270);
        DriveWithCorrection(-9.5, 270, 0.4);
        dropper.OpenDropper();
        sleep(1000);

    }

}
