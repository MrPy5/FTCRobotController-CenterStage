package org.firstinspires.ftc.teamcode.OpModes.Game.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Dropper;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Intake;

@TeleOp(name = "Motor Test")
public class MotorTest extends LinearOpMode {

    double triggerSensitivity = 0.01;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, true);
        //Lift lift = robot.new Lift();
        Intake intake = robot.new Intake();

        double speed = .3;

        ElapsedTime dpad = new ElapsedTime();
        dpad.startTime();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > triggerSensitivity) {
                robot.frontLeft.setPower(speed);
            } else {
                robot.frontLeft.setPower(0);
            }
            if (gamepad1.right_trigger > triggerSensitivity) {
                robot.frontRight.setPower(speed);
            } else {
                robot.frontRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                robot.backLeft.setPower(speed);
            } else {
                robot.backLeft.setPower(0);
            }
            if (gamepad1.right_bumper) {
                robot.backRight.setPower(speed);
            } else {
                robot.backRight.setPower(0);
            }
            if (gamepad1.dpad_left) {
                intake.StartIntake(speed);
            } else {
                intake.StopIntake();
            }
            /*if (gamepad1.dpad_right) {
                robot.liftMotor.setPower(speed);
            } else {
                robot.liftMotor.setPower(0);
            }*/

            if (gamepad1.dpad_up) {
                speed += 0.01;
            }

            if (gamepad1.dpad_down) {
                speed -= 0.01;

            }

            if (gamepad1.cross) {
                robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

            telemetry.addData("FrontLeft: ", robot.frontLeft.getCurrentPosition());

            telemetry.addData("FrontRight: ", robot.frontRight.getCurrentPosition());

            telemetry.addData("BackLeft: ", robot.backLeft.getCurrentPosition());

            telemetry.addData("BackRight: ", robot.backRight.getCurrentPosition());

            telemetry.addData("Intake: ", robot.intakeMotor.getCurrentPosition());

            //telemetry.addData("Lift: ", robot.liftMotor.getCurrentPosition());

            telemetry.addData("Speed: ", speed);

            telemetry.update();
        }
    }
}
