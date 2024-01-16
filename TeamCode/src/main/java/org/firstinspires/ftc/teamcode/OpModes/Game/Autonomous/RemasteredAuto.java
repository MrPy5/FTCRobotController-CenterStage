package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AnotherTestAuto")
public class RemasteredAuto extends AutoControlsRemastered {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";


        //int spikeLocation = 1;
        waitForStart();

        Motion driveOne = new Motion();
            /*driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            driveOne.add(new Drive(new MillisecondTrigger(0), 48, 0.7, 0));
            driveOne.add(new SpikeDrop(new InchesTrigger(24.5, driveOne, 1)));
            driveOne.add(new MoveHoist(new IndexTrigger(1, driveOne), hoist.stackPosition - 0.07));
            driveOne.add(new Drive(new IndexTrigger(1, driveOne), 22, 0.5, 90));
            driveOne.add(new MoveIntake(new IndexTrigger(4, driveOne), 1000));
            driveOne.add(new MoveIntake(new IndexTrigger(5, driveOne), 1500));
            driveOne.add(new Drive(new IndexTrigger(5, driveOne), -80, 0.8, 90));

            */

            //driveOne.add(new MoveHoist(new MillisecondTrigger(0), hoist.hoistedPosition));
            //driveOne.add(new Drive(new MillisecondTrigger(0), 26, 0.5, 0));
            //driveOne.add(new Turn(new IndexTrigger(1, driveOne), 90));
            //driveOne.add(new SpikeDrop(new IndexTrigger(2, driveOne)));
            driveOne.add(new CatWalk(new MillisecondTrigger(0), 30, 10, 0.3, 0, 0.5));


            //driveOne.add(new MoveHoist(-1, 2, hoist.stackPosition));
            //driveOne.add(new MoveIntake(-1, 3, 3000));*/
        driveOne.Start(0);

    }

}
