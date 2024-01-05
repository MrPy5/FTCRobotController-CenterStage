package org.firstinspires.ftc.teamcode.OpModes.Game.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Dropper;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Suspension;
import org.firstinspires.ftc.teamcode.Hardware.Robot.DroneLauncher;
import org.firstinspires.ftc.teamcode.Hardware.Robot.SpikeHook;
import org.firstinspires.ftc.teamcode.Hardware.Robot.PixelSplitter;
import org.firstinspires.ftc.teamcode.Hardware.Robot.pixelSplitterState;
import org.firstinspires.ftc.teamcode.Hardware.Robot.IntakeHoist;



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
        Suspension suspension = robot.new Suspension();
        DroneLauncher droneLauncher = robot.new DroneLauncher();
        SpikeHook spikeHook = robot.new SpikeHook();
        PixelSplitter pixelSplitter = robot.new PixelSplitter();
        IntakeHoist intakeHoist = robot.new IntakeHoist();


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
        double liftUpModifier = 1;

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
        spikeHook.ResetSpike();

        //---Suspension---//
        int suspensionToggle = 0;
        boolean suspensionReset = true;

        while (opModeIsActive()) {


            //---Gamepad1 controls---//
            // driving -> joysticks
            // intake -> Right trigger
            // outtake -> Left trigger
            // stop intake -> Right Bumper
            // ground -> D-Pad down
            // stack -> D-Pad left
            // hoisted -> D-Pad up
            // launch drone -> triangle

            //---Gamepad2 controls---//
            //
            // lift presets -> Right Bumper, a, b, y
            // lift position reset -> Left Bumper
            // manual lift -> D-Pad up & down
            // drop pixel -> Right trigger
            // toggle suspension activation -> Left Trigger
            // raise & lower suspension -> D-Pad left & right

            /*









             */




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

            if (gamepad1.left_trigger > robot.triggerSensitivity) {
                intake.StartIntake(-1);
            }
            if (gamepad1.right_trigger > robot.triggerSensitivity) {
                intake.StartIntake(1);
            }
            if (gamepad1.right_bumper) {
                intake.StopIntake();
            }

            if (gamepad1.square) {
                spikeHook.DropSpike();
            }
            if (gamepad1.circle) {
                spikeHook.ResetSpike();
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
            if (gamepad2.left_bumper) {
                robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            if (gamepad1.triangle) {
                droneLauncher.ReleaseDrone();
            }

            //---Pixel Splitter---//
            /*if (gamepad2.left_trigger > robot.triggerSensitivity) {
                if (pixelSplitter.pixelSplitterReset) {

                    if (pixelSplitter.splitterState == pixelSplitterState.Open) {
                        pixelSplitter.SplitPixel();
                        pixelSplitter.pixelSplitterReset = false;
                    }
                    if (pixelSplitter.splitterState == pixelSplitterState.Split) {
                        pixelSplitter.OpenSplitter();
                        pixelSplitter.pixelSplitterReset = false;
                    }

                }

            }
            else {
                pixelSplitter.pixelSplitterReset = true;
            }*/

            //---Suspension---//
            //Servo
            if (gamepad2.left_trigger > robot.triggerSensitivity && suspensionReset) {
                if (suspensionToggle == 0) {
                    suspension.ActivateSuspension();
                    suspensionToggle = 1;
                }
                else if (suspensionToggle == 1) {
                    suspension.DeactivateSuspension();
                    suspensionToggle = 0;
                }
                suspensionReset = false;
            }
            if (gamepad2.left_trigger < robot.triggerSensitivity) {
                suspensionReset = true;
            }
            //Motor
            if (gamepad2.dpad_right) {
                robot.suspensionMotor.setPower(1);
            }
            if (gamepad2.dpad_left) {
                robot.suspensionMotor.setPower(-1);
            }
            if (!gamepad2.dpad_right && !gamepad2.dpad_left) {
                robot.suspensionMotor.setPower(0);
            }

            //Hoist
            if (gamepad1.dpad_down) {
                intakeHoist.Ground();
            }
            if (gamepad1.dpad_left) {
                intakeHoist.Stack();
            }
            if (gamepad1.dpad_up) {
                intakeHoist.Hoist();
            }

            telemetry.addData("Manual Lift Mode: ", manualLiftMode);
            telemetry.addData("Timer: ", robot.gameTimer.seconds());
            telemetry.update();
        }
    }



}
