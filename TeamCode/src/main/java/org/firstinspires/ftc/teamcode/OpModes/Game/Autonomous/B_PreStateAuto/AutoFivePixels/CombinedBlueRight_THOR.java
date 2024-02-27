package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.AutoFivePixels;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;
@Disabled
@Autonomous(name = "Combined Blue Right - Thor", group = "Remastered")
public class CombinedBlueRight_THOR extends AutoControlsCombined {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

        double previousStrafe = 0;


        //Vision
        int spikeLocation = robot.ScanForElementBitmap(2);
        sleep(1000);
        while (opModeInInit()) {
            spikeLocation = robot.ScanForElementBitmap(2);
            telemetry.addData("Spike: ", spikeLocation);
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
        if (spikeLocation == 3) {
            //CHANGE FOR THOR
            sleep(2000);
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 38.5, 0.5, 0));
            driveOne.add(new Strafe(new IndexTrigger(1, driveOne), 4.5, 0.4, 0, -1));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTriggerWithDelay(3, 400, driveOne), 7, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(5, driveOne), -2, 0.4, 270));
        }

        else if (spikeLocation == 2) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Strafe(new MillisecondTrigger(0), 10, 0.5, 0, -1));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), 39, 0.6, 0));
            driveOne.add(new Strafe(new IndexTrigger(2, driveOne), 15, -0.4, 0, -1));

            driveOne.add(new SpikeDrop(new IndexTrigger(3, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTriggerWithDelay(5, 300, driveOne), 6, 0.35, 0));
            driveOne.add(new Drive(new IndexTrigger(6, driveOne), 4, 0.4, 270));


        }

        else if (spikeLocation == 1) {
            //Initial Spike drop
            //Take first pixel from stack
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 28, 0.5, 0));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), -5, 0.4, 270));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(4, driveOne), 4, 0.25, 270));
            //driveOne.add(new CatWalk(new IndexTrigger(5, driveOne), 23, 20.5, 0.4, 90, 1.2, new Vision()));
            driveOne.add(new Strafe(new IndexTrigger(5, driveOne), 19.5, -0.4, 270, -1));

            //driveOne.add(new MoveHoist(new IndexTrigger(6, driveOne), hoist.stackPosition5));
            //driveOne.add(new MoveIntake(new IndexTrigger(6, driveOne), 1500));
        }
        driveOne.Start(0);
        if (spikeLocation == 1) {
            DriveAtAngleToStack(20, 0.3);
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
        driveTwo.add(new Drive(new IndexTrigger(1, driveTwo), -100, 1, 270));

        //CHANGE FOR THOR
        if (spikeLocation == 3) {
            driveTwo.add(new MoveLift(new IndexTrigger(3, driveTwo), lift.liftLow + 2, 0));
        }
        else {
            driveTwo.add(new MoveLift(new IndexTrigger(3, driveTwo), lift.liftLow + 1, 0));

        }
        //driveTwo.add(new CatWalk(new IndexTrigger(1, driveTwo), -28, -28, 0.5, 90, 0.5, new Vision()));

        driveTwo.Start(0);
        hoist.Hoist();
        //Strafe on backboard
        if (spikeLocation == 1) {
            previousStrafe = StrafeWithInchesWithCorrection(31, 0.25, 1, 270);
        }
        if (spikeLocation == 2) {
            previousStrafe = StrafeWithInchesWithCorrection(26, 0.25, 2, 270);
        }
        if (spikeLocation == 3) {
            previousStrafe = StrafeWithInchesWithCorrection(21, 0.25, 3, 270);
        }
        DriveWithCorrection(-10, 270, 0.3);

        Motion driveThree = new Motion();
        //Drop pixel and move back to stack
        driveThree.add(new MoveDropper(new MillisecondTrigger(0), "open"));
        driveThree.add(new MoveLift(new IndexTriggerWithDelay(0, 300, driveThree), lift.liftLow + 5, lift.liftLow - 1));
        driveThree.add(new Drive(new IndexTriggerWithDelay(1, 300, driveThree), 5, 0.5, 270));
        driveThree.add(new MoveLift(new IndexTriggerWithDelay(2, 0, driveThree), lift.liftBottom, lift.liftLow + 4));
        driveThree.add(new MoveDropper(new IndexTriggerWithDelay(2, 0, driveThree), "close"));
        driveThree.add(new MoveHoist(new IndexTrigger(3, driveThree), hoist.stackPosition3));



        driveThree.Start(0);

        if (previousStrafe == -1) {
            if (spikeLocation == 1) {
                //driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, 18, 0.5, 90, 0.5, new Vision(), 1));
                StrafeWithInchesWithCorrection(30, -0.5, -1, 270);

                DriveWithCorrection(80, 270, 0.9);
            }
            if (spikeLocation == 2) {
                //driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, 23, 0.5, 90, 0.5, new Vision(), 1));
                StrafeWithInchesWithCorrection(23, -0.5, -1, 270);

                DriveWithCorrection(80, 270, 0.9);
            }
            if (spikeLocation == 3) {
                //driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 85, 29, 0.5, 90, 0.5, new Vision(), 1));
                StrafeWithInchesWithCorrection(17, -0.5, -1, 270);

                DriveWithCorrection(80, 270, 0.9);
            }
        } else {
            StrafeWithInchesWithCorrection(previousStrafe, -0.5, -1, 270);
            DriveWithCorrection(80, 270, 0.9);
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
        driveFour.add(new Drive(new MillisecondTrigger(1500), -100, 0.9, 270));

        driveFour.add(new MoveLift(new IndexTrigger(3, driveFour), lift.liftLow + 2, 0));



        driveFour.Start(0);
        //Strafe along board
        if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(25, 0.3, 2, 270);
        } else {
            StrafeWithInchesWithCorrection(21, 0.3, 3, 270);
        }

        DriveWithCorrection(-10, 270, 0.3);
        dropper.OpenDropper();
        sleep(500);
        lift.SetPosition(lift.liftLow + 9, lift.liftLow, -1);
        sleep(600);
    }

}
