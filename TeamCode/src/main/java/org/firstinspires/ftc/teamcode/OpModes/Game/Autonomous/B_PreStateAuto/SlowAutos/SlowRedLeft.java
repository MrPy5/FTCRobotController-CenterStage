package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.SlowAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.AutoControls;
@Disabled

@Autonomous(name = "Red Left", group = "Slow Autos")
public class SlowRedLeft extends AutoControls {

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
            telemetry.addData("White Three: ", robot.whiteThree);
            telemetry.update();
        }

        waitForStart();
        robot.gameTimer.startTime();

        //Position 1
        if (spikeLocation == 1) {
            /*
            DriveWithCorrection(30, 0, 0.3);
            sleep(500);
            Turn(270);
            sleep(500);
            DriveWithCorrection(-3, 270, 0.3);
            sleep(500);
            spike.DropSpike();
            sleep(1000);
            spike.ResetSpike();
            DriveWithCorrection(1, 270, 0.3);
            sleep(500);
            StrafeWithInchesWithCorrection(20, -0.2, -1, 270);
            sleep(500);
            Turn(90);
            sleep(500);
            DriveWithCorrection(-75, 90, 0.3);
            sleep(500);
            StrafeWithInchesWithCorrection(48, -0.3, 4, 90);
            DriveWithCorrectionToAprilTag(-20, 90, 0.4, 4);
             */

            DriveWithCorrection(42, 0, 0.3);
            sleep(500);
            StrafeWithInchesWithCorrection(14, -0.3, -1, 0);
            sleep(500);
            DriveWithCorrection(-1, 0, 0.3);
            sleep(500);
            spike.DropSpike();
            sleep(500);
            spike.ResetSpike();
            DriveWithCorrection(5, 0, 0.3);
            Turn(90);
            DriveWithCorrection(-88, 90, 0.3);
            sleep(500);
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

        lift.SetPosition(lift.liftLow - 4,  lift.liftAprilTags, -1);
        sleep(500);
        DriveWithCorrection(-2, 90, 0.3);
        sleep(500);
        dropper.OpenDropper();
        sleep(500);
        dropper.CloseDropper();
        lift.SetPosition(lift.liftLow, lift.liftLow - 5, -1);
        sleep(500);

        Drive(3);
        sleep(1000);
        lift.SetPosition(lift.liftBottom, lift.liftLow, -1);
        sleep(1000);


    }

}
