package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.RemasteredAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsRemastered;

@Autonomous(name = "Combined Red Left", group = "Remastered")
public class CombinedAuto extends AutoControlsCombined {

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

        }

        else if (spikeLocation == 2) {

        }

        else if (spikeLocation == 3) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 28, 0.5, 0));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), -5, 0.4, 90));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition + 0.07));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 5, 0.25, 90));
            driveOne.add(new CatWalk(new IndexTrigger(5, driveOne), 23, 18.5, 0.4, 90, 1.2, new Vision()));
            driveOne.add(new MoveIntake(new IndexTrigger(6, driveOne), 1500));
        }
        driveOne.Start(0);


       Motion driveTwo = new Motion();
            driveTwo.add(new MoveIntake(new MillisecondTrigger(0), 1500));
            driveTwo.add(new Drive(new MillisecondTrigger(0), -100, 0.8, 90));
            driveTwo.add(new MoveLift(new IndexTrigger(1, driveTwo), lift.liftLow - 2, 0));
            //driveTwo.add(new CatWalk(new IndexTrigger(1, driveTwo), -28, -28, 0.5, 90, 0.5, new Vision()));

        driveTwo.Start(0);

        StrafeWithInchesWithCorrection(20, -0.3, 6, 90);
        DriveWithCorrection(-10, 90, 0.4);

       Motion driveThree = new Motion();
            driveThree.add(new MoveDropper(new MillisecondTrigger(0), "open"));
            driveThree.add(new MoveLift(new IndexTriggerWithDelay(0, 2000, driveThree), lift.liftBottom, lift.liftLow - 2));
            driveThree.add(new Drive(new IndexTriggerWithDelay(0, 1000, driveThree), 10, 0.5, 90));
            driveThree.add(new MoveDropper(new IndexTriggerWithDelay(0, 2000, driveThree), "close"));
            driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 31, 28, 0.5, 90, 0.5, new Vision()));
            driveThree.add(new Drive(new IndexTrigger(4, driveThree), 40, 0.8, 90));

        driveThree.Start(0);

        //Hoist position set in function
        DriveWithCorrectionToStack(40, 90, 0.4, hoist.stackPosition + 0.03);
        DriveWithCorrection(25, 90, 0.4);

        Motion driveFour = new Motion();
            driveFour.add(new MoveIntake(new MillisecondTrigger(0), 4500));
            driveFour.add(new Drive(new MillisecondTrigger(1500), -100, 0.8, 90));
            driveFour.add(new MoveLift(new IndexTrigger(1, driveFour), lift.liftLow - 2, 0));

        driveFour.Start(0);

        StrafeWithInchesWithCorrection(28, -0.3, 4, 90);
        DriveWithCorrection(-10, 90, 0.4);
        dropper.OpenDropper();
        sleep(350);
        dropper.CloseDropper();

    }

}
