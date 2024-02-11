package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.FastAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsRemastered;
@Disabled
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
            driveOne.add(new Drive(new MillisecondTrigger(0), 35, 0.4, 0));
            driveOne.add(new CatWalk(new IndexTrigger(1, driveOne), 5, -21, 0.3, 0, 0.5, new Vision(), 0.5));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 4, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(5, driveOne), -11, 0.4, 90));
        }

        else if (spikeLocation == 2) {
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new CatWalk(new MillisecondTrigger(0), 40, -15, 0.5, 0, 0.5, new Vision(), 0.5));
            driveOne.add(new SpikeDrop(new IndexTrigger(1, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(2, driveOne), hoist.stackPosition4));
            driveOne.add(new Drive(new IndexTrigger(2, driveOne), 4, 0.25, 0));
            driveOne.add(new Drive(new IndexTrigger(4, driveOne), -8, 0.4, 90));


        }

        else if (spikeLocation == 3) {
            //Initial Spike drop
            //Take first pixel from stack
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 27.5, 0.5, 0));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), -4, 0.4, 90));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition5));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 7, 0.25, 90));
            //driveOne.add(new CatWalk(new IndexTrigger(5, driveOne), 23, 20.5, 0.4, 90, 1.2, new Vision()));
            driveOne.add(new CatWalk(new IndexTrigger(5, driveOne), 0, 19.5, 0.4, 90, 1.2, new Vision(), 0.5));

            //driveOne.add(new MoveHoist(new IndexTrigger(6, driveOne), hoist.stackPosition5));
            //driveOne.add(new MoveIntake(new IndexTrigger(6, driveOne), 1500));
        }
        driveOne.Start(0);

        Motion driveTwo = new Motion();



        //Drive back toward backboard
        driveTwo.add(new MoveIntake(new IndexTrigger(1, driveTwo), 2000));
        driveTwo.add(new Drive(new IndexTrigger(1, driveTwo), -96, 0.8, 90));
        driveTwo.add(new MoveLift(new IndexTrigger(3, driveTwo), lift.liftLow - 3, 0));
        //driveTwo.add(new CatWalk(new IndexTrigger(1, driveTwo), -28, -28, 0.5, 90, 0.5, new Vision()));

        driveTwo.Start(0);

        //Strafe on backboard
        if (spikeLocation == 1) {
            StrafeWithInchesWithCorrection(15, -0.25, 4, 90);
        }
        if (spikeLocation == 2) {
            StrafeWithInchesWithCorrection(26, -0.25, 5, 90);
        }
        if (spikeLocation == 3) {
            StrafeWithInchesWithCorrection(34, -0.25, 6, 90);
        }
        DriveWithCorrection(-14, 90, 0.4);

        Motion driveThree = new Motion();
        //Drop pixel and move back to stack
        driveThree.add(new MoveDropper(new MillisecondTrigger(0), "open"));
        driveThree.add(new MoveLift(new IndexTriggerWithDelay(0, 1500, driveThree), lift.liftBottom, lift.liftLow - 3));

        sleep(1000);

    }

}
