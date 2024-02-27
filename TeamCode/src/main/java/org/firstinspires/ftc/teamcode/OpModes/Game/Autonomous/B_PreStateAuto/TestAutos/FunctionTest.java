package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;
@Disabled
@Autonomous(name = "Function Test", group = "testAutos")
public class FunctionTest extends AutoControlsCombined {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";


        //Vision

        waitForStart();
        switchToContourPipeline();

        Motion driveOne = new Motion();

            //driveOne.add(new CatWalk(new MillisecondTrigger(0), 85, 24, 1, 0, 1, new Vision(), 1));
            driveOne.add(new Strafe(new MillisecondTrigger(0), 25, 0.5, 0, -1));
            driveOne.add(new Drive(new IndexTrigger(0, driveOne), 85, 1, 0));
        driveOne.Start(0);
        sleep(5000);

    }

}
