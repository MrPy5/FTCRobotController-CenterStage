package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;


@Autonomous(name = "WhiteTest", group = "fast")
public class WhiteTest extends AutoControlsCombined {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

        waitForStart();
        switchToContourPipeline();
        robot.gameTimer.startTime();
        lift.SetPosition(lift.liftLow, 0, -1);
        sleep(4000);
        StrafeWithInchesWithCorrection(40, 0.25, 2, 0);
        /*
        while (opModeIsActive()) {
            if (gamepad1.a) {
                // Temporarily stop the live view (RC preview).
                robot.visionPortal.stopLiveView();
                robot.webcam.stopStreaming();

                sleep(1000);
            }

            telemetry.addData("contours", robot.getCenter());

            AprilTagPoseFtc tag4 = robot.getTargetAprilTagPos(4);
            if (tag4 != null) {
                telemetry.addData("4x", tag4.x);
                telemetry.addData("4y", tag4.y);
                telemetry.addData("4z", tag4.z);
            }
            /*
            AprilTagPoseFtc tag5 = robot.getTargetAprilTagPos(5);
            if (tag5 != null) {
                telemetry.addData("5x", tag5.x);
                telemetry.addData("5y", tag5.y);
                telemetry.addData("5z", tag5.z);
            }

            AprilTagPoseFtc tag6 = robot.getTargetAprilTagPos(6);
            if (tag6 != null) {
                telemetry.addData("6x", tag6.x);
                telemetry.addData("6y", tag6.y);
                telemetry.addData("6z", tag6.z);
            }*//*
            telemetry.addData("fps", robot.visionPortal.getFps());
            telemetry.update();
        }*/



    }

}
