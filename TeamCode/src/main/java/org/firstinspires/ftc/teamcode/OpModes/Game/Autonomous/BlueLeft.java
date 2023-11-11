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



        if (spikeLocation == 3) {
            Drive(18);
            sleep(1000);
            Drive(7);
            sleep(500);
            Turn(300);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();


            Turn(270);
            Drive(-38);

            Turn(270);
            //Strafe(1000, 0);
            Drive(-5);
            lift.SetPosition(lift.liftLow - 4, 0);
            sleep(2000);
            dropper.OpenDropper();
            sleep(1000);
            dropper.CloseDropper();
            Drive(3);
            sleep(1000);
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

            Drive(-5);
            lift.SetPosition(lift.liftLow - 4.5, 0);
            sleep(2000);
            dropper.OpenDropper();
            sleep(1000);
            dropper.CloseDropper();
            Drive(3);
            sleep(1000);
            lift.SetPosition(lift.liftBottom, lift.liftLow);
            sleep(4000);


        }
        if (spikeLocation == 1) {
            Turn(90);
            Drive(24);
            sleep(500);
            Turn(0);
            Drive(26);
            Turn(270);
            Drive(1.5);
            intake.StartIntake(-0.7);
            sleep(800);
            intake.StopIntake();
            Drive(-12);
            sleep(500);
            Strafe(1150, 1);

            Drive(-3);
            lift.SetPosition(lift.liftLow - 4, 0);
            sleep(2000);
            dropper.OpenDropper();
            sleep(1000);
            dropper.CloseDropper();
            Drive(1.5);
            sleep(1000);
            lift.SetPosition(lift.liftBottom, lift.liftLow);
            sleep(4000);
        }





    }

}