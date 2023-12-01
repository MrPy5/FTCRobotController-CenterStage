package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test Auto")
public class TestAuto extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";


        waitForStart();
        robot.gameTimer.startTime();

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






    }

}
