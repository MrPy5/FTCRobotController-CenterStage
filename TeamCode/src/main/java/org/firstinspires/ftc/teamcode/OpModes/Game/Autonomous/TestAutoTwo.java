package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
@Disabled
@Autonomous(name = "TestAutoTWO")
public class TestAutoTwo extends AutoControlsCombined {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";
        switchToContourPipeline();
        waitForStart();
        sleep (5000);
        robot.intakeHoist.setPosition(hoist.stackPosition5);
        DriveAtAngleToStack(26, 0.35);  // Hit the stack
        //DriveShortDistance(-.2, .08); // back off a tiny bit
        intake.StartIntake(1);
        sleep(800);
        robot.intakeHoist.setPosition(hoist.stackPosition4);
        sleep(800);
        intake.StopIntake();
    }

}
