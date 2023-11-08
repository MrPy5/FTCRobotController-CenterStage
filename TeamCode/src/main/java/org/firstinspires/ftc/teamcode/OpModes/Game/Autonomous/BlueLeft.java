package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Left")
public class BlueLeft extends FirstMeetAutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

        int spikeLocation = robot.ScanForElement();
        while (opModeInInit()) {

            spikeLocation = robot.ScanForElement();
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.update();
        }
        waitForStart();
        robot.gameTimer.startTime();

        telemetry.addData("center", robot.getCenter());
        telemetry.update();

        //StrafeWithInches(10, 0);

        if (spikeLocation == 1) {
            Drive(18);
            sleep(1000);
            Drive(7);
            sleep(500);
            Turn(60);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Turn(1);
            Drive(-23);
            Turn(90);
            Drive(38);

            Turn(270);
            Strafe(750, 1);
            Drive(-5);
            lift.SetPosition(lift.liftLow - 3.5, 0);
            sleep(2000);
            dropper.OpenDropper();
            sleep(1000);
            dropper.CloseDropper();
            Drive(3);
            sleep(3000);
            lift.SetPosition(lift.liftBottom, lift.liftLow);
            sleep(4000);
        }
        if (spikeLocation == 2) {
            Drive(18);
            sleep(1000);
            Drive(3);

            sleep(500);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Turn(90);
            Drive(35);
            Turn(270);

            Strafe(500, 0);
            Drive(-5);
            lift.SetPosition(lift.liftLow - 3.5, 0);
            sleep(2000);
            dropper.OpenDropper();
            sleep(1000);
            dropper.CloseDropper();
            Drive(3);
            sleep(3000);
            lift.SetPosition(lift.liftBottom, lift.liftLow);
            sleep(4000);


        }
        if (spikeLocation == 3) {
            Turn(90);
            Drive(24);
            sleep(500);
            Turn(1);
            Drive(26);
            Turn(270);
            Drive(-5);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Drive(-12);
            sleep(500);
            Strafe(1050, 0);
            Turn(90);

            Drive(-3);
            lift.SetPosition(lift.liftLow - 3.5, 0);
            sleep(2000);
            dropper.OpenDropper();
            sleep(1000);
            dropper.CloseDropper();
            Drive(3);
            sleep(3000);
            lift.SetPosition(lift.liftBottom, lift.liftLow);
            sleep(4000);
        }




    }

}
