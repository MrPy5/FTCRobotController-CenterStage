/*package org.firstinspires.ftc.teamcode.OpModes.Game.TeleOp;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Dropper;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Intake;

import java.util.List;

@TeleOp(name = "GAME TELEOP")

public class GameTeleop extends LinearOpMode {


    @Override
    public void runOpMode() {

        //---Start Robot---//
        Robot robot = new Robot(hardwareMap, true);
        //Lift lift = robot.new Lift();
        Dropper dropper = robot.new Dropper();
        Intake intake = robot.new Intake();

        robot.initAprilTag();
        waitForStart();

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

        double avgWheelVelocityFPS;


        while (opModeIsActive()) {


            //---Gamepad1 controls---//
            // driving -> joysticks
            // intake -> rightBumper
            // outtake -> leftBumper

            //---Gamepad2 controls---//
            // lift -> bumpers
            // lift presets -> A, B, Y
            // drop -> rightTrigger


            //---Driving---//
            double leftStickY = gamepad1.left_stick_y * -1;
            double leftStickX = gamepad1.left_stick_x * 0.8; // testing


            double rightStickX = gamepad1.right_stick_x * .8;


            int frontLeft = robot.frontLeft.getCurrentPosition();
            int frontRight = robot.frontRight.getCurrentPosition();
            int backLeft = robot.backLeft.getCurrentPosition();
            int backRight = robot.backRight.getCurrentPosition();


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

            if (gamepad1.x) {

                if (intake.intakeReset) {
                    if (intake.intakeState == 0) {
                        intake.StartIntake(0.9);
                    }
                    else if (intake.intakeState == 1) {
                        intake.StopIntake();
                    }
                    intake.intakeReset = false;
                }
            }
            else {
                intake.intakeReset = true;
            }


            //---Lift---//
            //if (gamepad2.right_bumper) {
            //    lift.SetPosition(0);
           // }
           // if (gamepad2.y) {
           //     lift.SetPosition(16);
           // }

            //---Dropper---//
            if (gamepad2.right_trigger > robot.triggerSensitivity) {
                if (dropper.resetDropper) {
                    if (dropper.dropperOpen) {
                        dropper.CloseDropper();
                    }
                    else {
                        dropper.OpenDropper();
                    }
                    dropper.resetDropper = false;
                }
            }
            else {
                dropper.resetDropper = true;
            }
        }
    }



}
/*
 */