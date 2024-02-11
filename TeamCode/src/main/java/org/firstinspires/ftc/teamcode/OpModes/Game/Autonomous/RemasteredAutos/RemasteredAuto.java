package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.RemasteredAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsRemastered;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name = "Remastered Red Left", group = "Remastered")
public class RemasteredAuto extends AutoControlsRemastered {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";


        //int spikeLocation = 1;
        waitForStart();

       Motion driveOne = new Motion();
            driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 28, 0.5, 0));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), -5, 0.4, 90));
            driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new MoveHoist(new IndexTrigger(3, driveOne), hoist.stackPosition5));
            driveOne.add(new Drive(new IndexTrigger(3, driveOne), 5, 0.25, 90));
            driveOne.add(new CatWalk(new IndexTrigger(5, driveOne), 23, 18.5, 0.4, 90, 0.9, new Vision()));
            driveOne.add(new MoveIntake(new IndexTrigger(6, driveOne), 1500));
        driveOne.Start(0);


       Motion driveTwo = new Motion();
            driveTwo.add(new MoveIntake(new MillisecondTrigger(0), 1500));
            driveTwo.add(new Drive(new MillisecondTrigger(0), -95, 0.8, 90));
            driveTwo.add(new MoveLift(new IndexTrigger(1, driveTwo), lift.liftLow - 2, 0));
            driveTwo.add(new CatWalk(new IndexTrigger(1, driveTwo), -28, -28, 0.5, 90, 0.5, new Vision()));

        driveTwo.Start(0);

       Motion driveThree = new Motion();
            driveThree.add(new MoveDropper(new MillisecondTrigger(0), "open"));
            driveThree.add(new MoveLift(new IndexTriggerWithDelay(0, 2000, driveThree), lift.liftBottom, lift.liftLow - 2));
            driveThree.add(new Drive(new IndexTriggerWithDelay(0, 1000, driveThree), 10, 0.5, 90));
            driveThree.add(new MoveDropper(new IndexTriggerWithDelay(0, 2000, driveThree), "close"));
            driveThree.add(new CatWalk(new IndexTrigger(2, driveThree), 31, 28, 0.5, 90, 0.5, new Vision()));
            driveThree.add(new Drive(new IndexTrigger(4, driveThree), 60, 0.8, 90));

        driveThree.Start(0);


    }

}
