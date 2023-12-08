package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

@Autonomous(name = "New Auto")
public class NewAuto extends AutoControls {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        //Vision
        boolean spikeLocation = robot.ScanForElementBitmap(1);
        sleep(1000);
        while (opModeInInit()) {
            spikeLocation = robot.ScanForElementBitmap(1);
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.addData("Area: ", robot.whiteOne);
            telemetry.addData("Area: ", robot.whiteTwo);
            telemetry.update();
        }

        waitForStart();
        robot.gameTimer.startTime();

        //Position 1
        if (spikeLocation == true) {
            DriveWithCorrection(18, 0, 0.2);
            sleep(500);
        }


    }

}
