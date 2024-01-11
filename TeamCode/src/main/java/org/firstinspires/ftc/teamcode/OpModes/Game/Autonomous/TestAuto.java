package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Autonomous(name = "TestAuto")
public class TestAuto extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        //Vision
        int spikeLocation = robot.ScanForElementBitmap(2);
        sleep(1000);
        while (opModeInInit()) {
            spikeLocation = robot.ScanForElementBitmap(2);
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.addData("White One: ", robot.whiteOne);
            telemetry.addData("White Two: ", robot.whiteTwo);
            telemetry.update();
        }

        //int spikeLocation = 1;
        waitForStart();
        switchToContourPipeline();

        robot.gameTimer.startTime();

        //Position 1
        if (spikeLocation == 1) {


            DriveWithCorrection(42, 0, 0.4);
            sleep(100);
            StrafeWithInchesWithCorrection(14, -0.4, -1, 0);
            sleep(100);
            DriveWithCorrection(-1, 0, 0.3);
            sleep(100);
            spike.DropSpike();
            sleep(400);
            spike.ResetSpike();
            DriveWithCorrection(5, 0, 0.4);
            Turn(90);
            DriveWithCorrection(-88, 90, 0.6);
            sleep(100);
            StrafeWithInchesWithCorrection(48, -0.3, 4, 90);
            DriveWithCorrectionToAprilTag(-40, 90, 0.4, 4);

        }
        //Position 2
        if (spikeLocation == 2) {
            /*
            DriveWithCorrection(18, 0, 0.2);
            sleep(500);
            DriveWithCorrection(27, 0, 0.2);
            sleep(500);
            spike.DropSpike();
            sleep(1000);
            spike.ResetSpike();
            DriveWithCorrection(2, 0, 0.2);
            sleep(250);
            Turn(90);
            DriveWithCorrection(-72, 90, 0.4);
            StrafeWithInchesWithCorrection(48, -0.2, 5, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 5);*/
            DriveWithCorrection(18, 0, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(10, -0.2, -1, 0);
            sleep(500);
            DriveWithCorrection(25.5, 0, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(2, 0.2, -1, 0);
            sleep(500);
            spike.DropSpike();
            sleep(500);
            spike.ResetSpike();
            DriveWithCorrection(3, 0, 0.2);
            sleep(500);
            Turn(90);
            DriveWithCorrection(-82, 90, 0.2);
            StrafeWithInchesWithCorrection(48, -0.2, 5, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 5);
        }
        //Position 3
        if (spikeLocation == 3) {
            DriveWithCorrection(28, 0, 0.3);
            sleep(500);
            Turn(90);
            sleep(500);
            DriveWithCorrection(-3, 90, 0.3);
            spike.DropSpike();
            sleep(1000);
            spike.ResetSpike();
            DriveWithCorrection(2, 90, 0.3);
            sleep(500);
            StrafeWithInchesWithCorrection(20, 0.2, -1, 90);
            sleep(500);
            DriveWithCorrection(-75, 90, 0.3);
            sleep(500);
            StrafeWithInchesWithCorrection(48, -0.2, 6, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 6);
        }

        lift.SetPosition(lift.liftLow - 5,  lift.liftAprilTags);
        sleep(200);
        DriveWithCorrection(-2, 90, 0.3);
        sleep(50);
        dropper.OpenDropper();
        sleep(600);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow, lift.liftLow - 5);
        sleep(50);

        Drive(3);
        sleep(200);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(500);
        StrafeWithInchesWithCorrection(18, 0.4, -1, 90);
        DriveWithCorrection(78, 90, 0.6);
        DriveWithCorrectionToStack(27, 90, 0.3);
        hoist.Stack();
        DriveWithCorrectionToStack(4, 90, 0.2);
        intake.StartIntake(1);
        sleep(1750);
        intake.StopIntake();
        DriveWithCorrection(-4, 90, 0.2);
        hoist.Ground();

        intake.StartIntake(0.6);
        DriveWithCorrection(-83, 90, 0.6);
        intake.StopIntake();

        StrafeWithInchesWithCorrection(48, -0.3, 4, 90);
        DriveWithCorrectionToAprilTag(-50, 90, 0.4, 4);

        lift.SetPosition(lift.liftLow - 1,  lift.liftAprilTags);
        sleep(300);
        DriveWithCorrection(-2, 90, 0.3);
        sleep(50);
        dropper.OpenDropper();
        sleep(600);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow + 2, lift.liftLow - 5);
        sleep(50);

        Drive(3);
        sleep(200);
        lift.SetPosition(lift.liftBottom, lift.liftLow);
        sleep(500);


    }

}
