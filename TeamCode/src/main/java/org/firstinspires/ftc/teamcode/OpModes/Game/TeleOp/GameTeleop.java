package org.firstinspires.ftc.teamcode.OpModes.Game.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Dropper;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot.DroneLauncher;

import java.util.List;

@TeleOp(name = "GAME TELEOP")

public class GameTeleop extends LinearOpMode {


    @Override
    public void runOpMode() {

        //---Start Robot---//
        Robot robot = new Robot(hardwareMap, true);
        Lift lift = robot.new Lift();
        Dropper dropper = robot.new Dropper();
        Intake intake = robot.new Intake();
        DroneLauncher droneLauncher = robot.new DroneLauncher();



        waitForStart();

        robot.gameTimer.startTime();

        //---Bulk Reads---//
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }



        //--Driving--//
        double sinAngleRadians;
        double cosAngleRadians;
        double factor;
        double wheelPower;
        double stickAngleRadians;
        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        //---Lift---//
        boolean manualLiftMode = false;
        double liftCurrent = 0;
        double liftPast = 0;
        double lastManualIncrement = 0;

        //---Init Motion---//
        robot.pixelDropper.setPosition(dropper.closedDropper);
        intake.StopIntake();
        lift.SetPosition(0, 0);
        dropper.CloseDropper();
        droneLauncher.Reset();


        while (opModeIsActive()) {


            //---Gamepad1 controls---//
            // driving -> joysticks
            // intake -> Right trigger
            // outtake -> Left trigger
            // stop intake -> Right Bumper
            // shoot drone -> D-Pad up

            //---Gamepad2 controls---//
            //
            // lift presets -> Right Bumper, a, b, y
            // D-Pad down -> Manual lift mode
            // manual lift -> Bumpers
            // Pixel Dropper -> Right trigger



            //---Driving---//
            double leftStickY = gamepad1.left_stick_y * -1;
            double leftStickX = gamepad1.left_stick_x * 0.8; // testing


            double rightStickX = gamepad1.right_stick_x * .8;

            /*
            int frontLeft = robot.frontLeft.getCurrentPosition();
            int frontRight = robot.frontRight.getCurrentPosition();
            int backLeft = robot.backLeft.getCurrentPosition();
            int backRight = robot.backRight.getCurrentPosition();
            */

            //---Driving Code---//

            wheelPower = Math.hypot(leftStickX, leftStickY);
            if (wheelPower > robot.deadStickZone) {

                wheelPower = ((1 - robot.wheelPowerMinToMove) * wheelPower + robot.wheelPowerMinToMove);

            } else {
                wheelPower = 0;
            }


            stickAngleRadians = Math.atan2(leftStickY, leftStickX);

            stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

            sinAngleRadians = Math.sin(stickAngleRadians);
            cosAngleRadians = Math.cos(stickAngleRadians);
            factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

            lfPower = wheelPower * cosAngleRadians * factor + rightStickX;
            rfPower = wheelPower * sinAngleRadians * factor - rightStickX;
            lrPower = wheelPower * sinAngleRadians * factor + rightStickX;
            rrPower = wheelPower * cosAngleRadians * factor - rightStickX;


            robot.backLeft.setPower(lrPower);
            robot.backRight.setPower(rrPower);
            robot.frontLeft.setPower(lfPower);
            robot.frontRight.setPower(rfPower);

            //---Intake---//

            if (gamepad1.right_trigger > robot.triggerSensitivity) {
                intake.StartIntake(1);
            }
            if (gamepad1.left_trigger > robot.triggerSensitivity) {
                intake.StartIntake(-1);
            }
            if (gamepad1.right_bumper) {
                intake.StopIntake();
            }



            //---Lift---//


            //Preset Positions
            if (gamepad2.right_bumper) {
                lift.SetPosition(lift.liftBottom, liftCurrent);
                liftCurrent = lift.liftBottom;
                dropper.CloseDropper();
            }
            if (gamepad2.cross) {
                lift.SetPosition(lift.liftLow, liftCurrent);
                liftCurrent = lift.liftLow;
            }
            if (gamepad2.circle) {
                lift.SetPosition(lift.liftMedium, liftCurrent);
                liftCurrent = lift.liftMedium;
            }
            if (gamepad2.triangle) {
                lift.SetPosition(lift.liftHigh, liftCurrent);
                liftCurrent = lift.liftHigh;
            }

            //Manual Control
            if (gamepad2.dpad_up && lastManualIncrement + 250 < robot.gameTimer.milliseconds()) {

                lift.SetPosition(liftCurrent + 3, liftCurrent);
                liftCurrent = liftCurrent + 3;
                lastManualIncrement = robot.gameTimer.milliseconds();
            }
            if (gamepad2.dpad_down && lastManualIncrement + 250 < robot.gameTimer.milliseconds()) {

                lift.SetPosition(liftCurrent - 3, liftCurrent);
                liftCurrent = liftCurrent - 3;
                lastManualIncrement = robot.gameTimer.milliseconds();
            }


            //---Dropper---//
            if (gamepad2.right_trigger > robot.triggerSensitivity) {
                dropper.OpenDropper();
            }

            //---Drone---//
            if (gamepad1.dpad_up && robot.gameTimer.seconds() > 120) {
                droneLauncher.ReleaseDrone();
            }

            telemetry.addData("Manual Lift Mode: ", manualLiftMode);
            telemetry.addData("Timer: ", robot.gameTimer.milliseconds());
            telemetry.addData("LiftTarget: ", liftCurrent);
            telemetry.update();
        }
    }



}
