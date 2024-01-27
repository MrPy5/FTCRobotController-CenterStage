package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "WhiteTest")
public class WhiteTest extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        //Vision
        robot.ScanForElement(1);

        while (opModeInInit()) {
            telemetry.addData("Width", robot.objectWidth);
            telemetry.addData("Height", robot.objectHeight);
            telemetry.update();
        }

        waitForStart();

        robot.gameTimer.startTime();



    }

}
