package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous.B_PreStateAuto.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

            angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double currentHeading = (360 + angles.firstAngle) % 360;

            telemetry.addData("heading", currentHeading);
            telemetry.update();

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
