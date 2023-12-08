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
        int spikeLocation = robot.ScanForElement(1);
        sleep(1000);
        while (opModeInInit()) {
            spikeLocation = robot.ScanForElement(1);
            telemetry.addData("Spike: ", spikeLocation);
            telemetry.addData("Area: ", robot.area);
            telemetry.update();
        }

        waitForStart();
        robot.gameTimer.startTime();

        //Position 1
        if (spikeLocation == 2) {
            DriveWithCorrection(18, 0, 0.2);
            sleep(500);
        }


    }

}
