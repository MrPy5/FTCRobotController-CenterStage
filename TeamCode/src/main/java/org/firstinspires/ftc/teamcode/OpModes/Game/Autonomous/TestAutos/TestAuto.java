package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControlsCombined;


@Autonomous(name = "TestAuto")
public class TestAuto extends AutoControlsCombined {

    @Override
    public void runOpMode() {
        initMethods(hardwareMap);
        robot.alliance = "red";

        double previousStrafe = 0;

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

        waitForStart();

        //Distance Sensor Telemetry
        while (opModeIsActive()) {

            StrafeFromDistanceSensor(0.25, 0);

        }

        /*
        //Strafe Testing
        previousStrafe = StrafeWithInchesWithCorrection(30, -0.25, 5, 0);

        telemetry.addData("InchesStrafed: ", previousStrafe);
        telemetry.update();

        sleep(2000);

        StrafeWithInchesWithCorrection(previousStrafe, 0.25, -1, 0);
        */

        /*
        StrafeWithInchesWithCorrection(21, 0.8, -1, 0);
        DriveWithCorrection(80, 0, 1);
        sleep(2000);
        */

    }

}
