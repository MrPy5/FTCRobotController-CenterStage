package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.AutoControls;

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

            telemetry.addData("Left Distance: ", robot.leftDS.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Distance: ", robot.rightDS.getDistance(DistanceUnit.INCH));
            telemetry.addData("Front Distance: ", robot.frontDS.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }

        //Strafe Testing
        /*
        previousStrafe = StrafeWithInchesWithCorrection(30, -0.25, 5, 0);

        telemetry.addData("InchesStrafed: ", previousStrafe);
        telemetry.update();

        sleep(2000);

        StrafeWithInchesWithCorrection(previousStrafe, 0.25, -1, 0);
         */



    }

}
