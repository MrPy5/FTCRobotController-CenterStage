package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test Auto")
public class TestAuto extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";


        waitForStart();
        robot.gameTimer.startTime();

        DriveWithCorrection(80, 0, 0.3);

        /*
        while (opModeIsActive()) {
            if (robot.getTargetAprilTagPos(2) != null) {
                telemetry.addData("hi", "true");
            }
            else {
                telemetry.addData("hi", "false");
            }
            telemetry.addData("First", robot.getFirstAprilTagID());
            telemetry.update();

            if (gamepad1.a) {
                while (robot.getTargetAprilTagPos(2) == null) {

                }
            }
        }
         */

    }

}
