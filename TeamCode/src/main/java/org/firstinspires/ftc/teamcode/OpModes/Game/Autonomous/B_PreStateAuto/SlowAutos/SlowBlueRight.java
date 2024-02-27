package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.SlowAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Disabled
@Autonomous(name = "Blue Right", group = "Slow Autos")
public class SlowBlueRight extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "blue";

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
        //sleep(2000);
        //Position 1
        if (spikeLocation == 1) {
            DriveWithCorrection(39, 0, 0.2);
            sleep(500);
            Turn(270);
            sleep(500);
            DriveWithCorrection(-2, 270, 0.2);
            sleep(500);
            spike.DropSpike();
            sleep(1000);
            spike.ResetSpike();
            DriveWithCorrection(3, 270, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(11, -0.2, -1, 270);
            sleep(500);
            DriveWithCorrection(-75, 270, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(48, 0.2, 1, 270);
            DriveWithCorrectionToAprilTag(-20, 270, 0.4, 1);
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
            Turn(270);
            DriveWithCorrection(-72, 270, 0.2);
            StrafeWithInchesWithCorrection(48, 0.2, 2, 270);
            DriveWithCorrectionToAprilTag(-20, 270, 0.4, 2);*/
            DriveWithCorrection(18, 0, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(10, 0.2, -1, 0);
            sleep(500);
            DriveWithCorrection(26.5, 0, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(12, -0.2, -1, 0);
            sleep(500);
            spike.DropSpike();
            sleep(500);
            spike.ResetSpike();
            DriveWithCorrection(3, 0, 0.2);
            sleep(500);
            Turn(270);
            DriveWithCorrection(-72, 270, 0.2);
            StrafeWithInchesWithCorrection(48, 0.2, 2, 270);
            DriveWithCorrectionToAprilTag(-20, 270, 0.4, 2);

        }
        //Position 3
        if (spikeLocation == 3) {
            /*
            DriveWithCorrection(30, 0, 0.2);
            sleep(500);
            Turn(90);
            sleep(500);
            DriveWithCorrection(-2, 90, 0.2);
            sleep(500);
            spike.DropSpike();
            sleep(1000);
            spike.ResetSpike();
            DriveWithCorrection(2, 90, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(22, 0.2, -1, 90);
            sleep(500);
            Turn(270);
            sleep(500);
            DriveWithCorrection(-72, 270, 0.2);
            sleep(500);
            StrafeWithInchesWithCorrection(48, 0.2, 3, 270);
            DriveWithCorrectionToAprilTag(-20, 270, 0.4, 3);
             */

            DriveWithCorrection(42, 0, 0.3);
            sleep(500);
            StrafeWithInchesWithCorrection(5, 0.3, -1, 0);
            sleep(500);
            spike.DropSpike();
            sleep(500);
            spike.ResetSpike();
            DriveWithCorrection(4, 0, 0.3);
            Turn(270);
            DriveWithCorrection(-78, 270, 0.3);
            sleep(500);
            StrafeWithInchesWithCorrection(48, 0.3, 3, 270);
            DriveWithCorrectionToAprilTag(-40, 270, 0.4, 3);

        }

        lift.SetPosition(lift.liftLow - 4,  lift.liftAprilTags, -1);
        sleep(500);
        DriveWithCorrection(-2, 270, 0.3);
        sleep(500);
        dropper.OpenDropper();
        sleep(500);
        dropper.CloseDropper();
        sleep(500);
        lift.SetPosition(lift.liftLow, lift.liftLow - 5, -1);
        sleep(500);

        Drive(3);
        sleep(1000);
        lift.SetPosition(lift.liftBottom, lift.liftLow, -1);
        sleep(1000);

    }

}
