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
            driveOne.add(new Drive(0, -1, 40, 0.5, 0));
            driveOne.add(new SpikeDrop(1250,-1));
            driveOne.add(new Drive(-1, 0, 20, 0.5, 90));
            driveOne.add(new MoveHoist(-1, 2, hoist.stackPosition));
            driveOne.add(new MoveIntake(-1, 3, 3000));
        driveOne.Start(5000);

    }

}
