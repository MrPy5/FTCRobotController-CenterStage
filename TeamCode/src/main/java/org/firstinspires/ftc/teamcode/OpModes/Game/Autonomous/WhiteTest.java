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
        switchToContourPipeline();

        while (opModeInInit()) {
            telemetry.addData("Width", robot.objectWidth);
            telemetry.addData("Height", robot.objectHeight);
            telemetry.update();
        }

        waitForStart();

        robot.gameTimer.startTime();
        DriveWithCorrectionToStack(40, 0, 0.2);
        //DriveAfterStrafeStack(0, 0.3);

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) robot.centerX + ", " + (int) robot.centerY + ")");
            telemetry.addData("Width", robot.objectWidth);
            telemetry.addData("Height", robot.objectHeight);
            telemetry.addData("inches", robot.convertPixelsToInches(robot.objectWidth));
            double h = robot.getDistance(robot.objectWidth);
            telemetry.addData("Distance in Inch", h);
            telemetry.addData("Distance from Robot", robot.getDistanceFromRobot(h));
            telemetry.update();
        }


    }

}
