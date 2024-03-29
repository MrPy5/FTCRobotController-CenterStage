package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Dropper;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.IntakeHoist;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot.SpikeHook;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

public abstract class AutoControlsCombined extends LinearOpMode {

    public Robot robot;
    public Dropper dropper;
    public Lift lift;
    public Intake intake;
    public SpikeHook spike;
    public IntakeHoist hoist;
    //public BNO055IMU imu;
    public BHI260IMU imu;
    public Orientation angles;

    public double DISTANCE_TOLERANCE = 0.5;
    public final double TAG_DISTANCE = 10;

    public ElapsedTime gameTimer = new ElapsedTime();

    public void resetZeroes() {
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void initMethods(HardwareMap hwMap) {
        robot = new Robot(hwMap, false);
        dropper = robot.new Dropper();
        lift = robot.new Lift();
        intake = robot.new Intake();
        spike = robot.new SpikeHook();

        hoist = robot.new IntakeHoist();

        spike.ResetSpike();
        dropper.CloseDropper();

        robot.initAprilTag();
        robot.initEasyOpenCV();

        initIMU();
        resetZeroes();
    }


    public void initIMU() {
        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        */

        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));


        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void switchToContourPipeline() {
        robot.switchPipeline();
    }
    public void ResetEncoders() {

        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public double GetAverageWheelPositionInches() {
        return ((((robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4.0) / robot.ticksPerInch));
    }
    public double GetAverageStrafePositionInches() {
        return (robot.frontLeft.getCurrentPosition() - robot.frontRight.getCurrentPosition() - robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition() / 4.0) / robot.strafeTicksPerInch;
    }

    public double GetAverageVelocity() {
        double averageVelocity;
        averageVelocity = (robot.backRight.getVelocity() + robot.backLeft.getVelocity() + robot.frontLeft.getVelocity() + robot.frontRight.getVelocity()) / 4;
        averageVelocity = (averageVelocity / robot.ticksPerInch) / 12;
        return averageVelocity;
    }

    public double GetLowestWheelVelocity() {
        double lowestWheelVelocity = Math.min(robot.backRight.getVelocity(), Math.min(robot.backLeft.getVelocity(), Math.min(robot.frontLeft.getVelocity(), robot.frontRight.getVelocity())));
        lowestWheelVelocity = (lowestWheelVelocity / robot.ticksPerInch) / 12;
        return lowestWheelVelocity;
    }


    public enum MoveState {
        Dead,
        Init,
        Finished,

    }
    public double GetTicks() {
        return (robot.frontLeft.getCurrentPosition() - robot.frontRight.getCurrentPosition() - robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition() / 4.0);
    }
    public void Strafe(double power) {
        double target = 6000;
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.backRight.setPower(power);

        while (opModeIsActive() && GetTicks() < target) {
            telemetry.addData("ticks", GetTicks());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        sleep(2000);
        telemetry.addData("ticks", GetTicks());
        telemetry.update();
        sleep(10000);

    }
    public double headingAdjustment(double targetHeading, double speedModifier) {
        double adjustment;
        double currentHeading;
        double degreesOff;
        boolean goRight;

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        currentHeading = (360 + angles.firstAngle) % 360;

        goRight = targetHeading > currentHeading;
        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }

        double speedMinimum;

    /*
        if (degreesOff > 10) {
            speedModifier = 6;
        }

     */

        speedModifier = 5;
        speedMinimum = 3;

        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff) / speedModifier, 2) + speedMinimum) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }

        return adjustment;
    }
    public double degreesOff(double targetHeading) {
        if (targetHeading == -1) {
            return 0;
        }
        double currentHeading;
        double degreesOff;

        angles   = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        currentHeading = (360 + angles.firstAngle) % 360;

        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {

            degreesOff = 360 - degreesOff;
        }

        return degreesOff;
    }

    // MOTION //
    //
    //
    //
    public class Motion {
        List<Move> moves = new ArrayList<>();

        public Motion() {

        }
        public void add(Move move) {
            moves.add(move);
        }
        public void Start(int millisecondDelay) {
            sleep(millisecondDelay);
            gameTimer.reset();
            boolean done = false;
            while (opModeIsActive() && !done) {
                telemetry.addData("time: ", gameTimer.milliseconds());


                int totalFinished = 0;

                for (Move move : moves) {
                    //init
                    if (move.trigger.CheckTrigger()) {
                        if (move.state == MoveState.Dead) {
                            move.Init();
                        }

                    }

                    //check
                    if (move.whileChecked && move.state == MoveState.Init) {
                        move.Check();
                    }
                    if (move.state == MoveState.Finished) {
                        totalFinished += 1;
                    }

                    if (moves.size() == (totalFinished)) {
                        done = true;
                    }
                }

                telemetry.update();
            }

        }
    }




    //TRIGGER//
    //
    //
    //
    public class Trigger {

        public Trigger() {

        }
        public boolean CheckTrigger() {
            return true;
        }
    }
    public class IndexTrigger extends Trigger {
        int index;
        Motion drive;
        public IndexTrigger(int indexPARAM, Motion drivePARAM) {
            index = indexPARAM;
            drive = drivePARAM;
        }
        public boolean CheckTrigger() {
            boolean value = drive.moves.get(index).state == MoveState.Finished;
            return value;
        }

    }
    public class IndexTriggerWithDelay extends Trigger {
        int index;
        int delayMillisecond;
        Motion drive;

        ElapsedTime triggerTimer = new ElapsedTime();
        int started = 0;
        public IndexTriggerWithDelay(int indexPARAM, int delayMillisecondPARAM, Motion drivePARAM) {
            index = indexPARAM;
            delayMillisecond = delayMillisecondPARAM;
            drive = drivePARAM;
        }
        public boolean CheckTrigger() {
            if (drive.moves.get(index).state == MoveState.Finished && started == 0) {
                started = 1;
            }
            if (started == 1) {
                triggerTimer.reset();
                started = 2;
            }
            boolean value = drive.moves.get(index).state == MoveState.Finished && triggerTimer.milliseconds() > delayMillisecond;
            return value;
        }

    }
    public class MillisecondTrigger extends Trigger {
        int millisecond;
        public MillisecondTrigger(int millisecondPARAM) {
            millisecond = millisecondPARAM;
        }
        public boolean CheckTrigger() {
            boolean value = (millisecond < gameTimer.milliseconds());
            return value;
        }

    }

    // MOVE //
    //
    //
    //
    public class Move {
        Trigger trigger;
        boolean whileChecked = false;
        MoveState state = MoveState.Dead;




        public Move(Trigger triggerPARAM, boolean whileCheckedPARAM) {
            trigger = triggerPARAM;
            whileChecked = whileCheckedPARAM;


        }
        public void Init() {

        }
        public void Check() {}
    }

    public class SpikeDrop extends Move {

        public SpikeDrop(Trigger triggerPARAM) {
            super(triggerPARAM, false);
        }

        public void Init() {
            state = MoveState.Init;
            spike.DropSpike();
            state = MoveState.Finished;
        }

    }
    public class MoveHoist extends Move {
        double hoistPosition;
        public MoveHoist(Trigger triggerPARAM, double hoistPositionPARAM) {
            super(triggerPARAM, false);
            hoistPosition = hoistPositionPARAM;
        }

        public void Init() {
            state = MoveState.Init;
            robot.intakeHoist.setPosition(hoistPosition);
            state = MoveState.Finished;
        }

    }

    public class MoveIntake extends Move {
        double moveIntakeForMilliseconds;

        ElapsedTime moveTimer = new ElapsedTime();
        public MoveIntake(Trigger triggerPARAM, double moveIntakeForMillisecondsPARAM) {
            super(triggerPARAM, true);
            moveIntakeForMilliseconds = moveIntakeForMillisecondsPARAM;
        }

        public void Init() {
            state = MoveState.Init;
            intake.StartIntake(0.9);
            moveTimer.reset();

        }
        public void Check() {
            if (moveTimer.milliseconds() > moveIntakeForMilliseconds) {
                intake.StopIntake();
                state = MoveState.Finished;
            }
        }

    }

    public class Drive extends Move {
        double targetInches;

        double minPower = 0.3;
        double targetPower;
        double power = minPower;

        double headingAdjustmentMultiplier = 2.5;
        double targetHeading;
        public Drive(Trigger triggerPARAM, double targetInchesPARAM, double powerPARAM, double targetHeadingPARAM) {
            super(triggerPARAM, true);
            targetInches = targetInchesPARAM;
            targetPower = powerPARAM;
            targetHeading = targetHeadingPARAM;
        }

        public void Init() {
            state = MoveState.Init;
            ResetEncoders();

            double currentInches;
            double distanceToTarget;



            double lfPower = power;
            double rfPower = power;
            double lrPower = power;
            double rrPower = power;

            double reverse;

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            if (Math.abs(distanceToTarget) > DISTANCE_TOLERANCE && opModeIsActive()) {

                double turnAdjustment;
                turnAdjustment = headingAdjustment(targetHeading, 0);

                currentInches = GetAverageWheelPositionInches();
                distanceToTarget = targetInches - currentInches;

                if (distanceToTarget < 0) {
                    reverse = -1;
                } else {
                    reverse = 1;
                }

                robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment);
                robot.frontRight.setPower((rfPower * reverse) - turnAdjustment);
                robot.backRight.setPower((rrPower * reverse) - turnAdjustment);
                robot.backLeft.setPower((lrPower * reverse) + turnAdjustment);
            }

        }
        public void Check() {


            double currentInches;
            double distanceToTarget;

            double reverse;
            double distanceSensorTolerance = 6;
            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            //telemetry.addData("compare", currentInchesCompare);
            //telemetry.update();
            //acceleration
            if (Math.abs(targetInches) > 70 && Math.abs(distanceToTarget) / Math.abs(targetInches) < 0.25) {
                power -= 0.05;
                if (power < minPower) {
                    power = minPower;
                }
            }
            else {
                power += 0.05;
                if (power > targetPower) {
                    power = targetPower;
                }

            }


            while (robot.backLeftDS.getDistance(DistanceUnit.INCH) < distanceSensorTolerance && robot.backRightDS.getDistance(DistanceUnit.INCH) < distanceSensorTolerance && Math.abs(targetInches) > 70 && opModeIsActive()) {

                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);

            }


            if (Math.abs(distanceToTarget) > DISTANCE_TOLERANCE && opModeIsActive()) {

                double turnAdjustment;
                turnAdjustment = headingAdjustment(targetHeading, 0);

                angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                double currentHeading = (360 + angles.firstAngle) % 360;

                telemetry.addData("headingAdjustment: ", turnAdjustment);
                telemetry.addData("heading: ", currentHeading);
                telemetry.update();

                currentInches = GetAverageWheelPositionInches();
                distanceToTarget = targetInches - currentInches;

                if (distanceToTarget < 0) {
                    reverse = -1;
                } else {
                    reverse = 1;
                }
                //recalculate turn adjustment based on speed
                turnAdjustment = turnAdjustment * power * headingAdjustmentMultiplier;
                robot.frontLeft.setPower(power * reverse + turnAdjustment);
                robot.frontRight.setPower(power * reverse - turnAdjustment);
                robot.backRight.setPower(power * reverse - turnAdjustment);
                robot.backLeft.setPower(power * reverse + turnAdjustment);
            }
            else {
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
                state = MoveState.Finished;
            }

        }

    }
    public class Turn extends Move {
        double targetHeading;
        double lastTurnTime = -1;
        boolean stalledTurn = false;
        double lastAngle = -1;

        public Turn(Trigger triggerPARAM, double targetHeadingPARAM) {
            super(triggerPARAM, true);
            targetHeading = targetHeadingPARAM;
        }

        public void Init() {

            state = MoveState.Init;
            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;
            double currentHeading;


            if (degreesOff(targetHeading) > 1 && opModeIsActive()) {

                double adjustment = 0;
                adjustment = headingAdjustment(targetHeading, 0);

                lfPower = adjustment;
                rfPower = -adjustment;
                lrPower = adjustment;
                rrPower = -adjustment;

                robot.frontLeft.setPower(lfPower);
                robot.frontRight.setPower(rfPower);
                robot.backLeft.setPower(lrPower);
                robot.backRight.setPower(rrPower);


                angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                currentHeading = (360 + angles.firstAngle) % 360;

                lastAngle = currentHeading;

            }

        }
        public void Check() {

            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;
            double currentHeading;

            if (degreesOff(targetHeading) > 1 && opModeIsActive()) {

                double adjustment = 0;
                adjustment = headingAdjustment(targetHeading, 0);

                lfPower = adjustment;
                rfPower = -adjustment;
                lrPower = adjustment;
                rrPower = -adjustment;

                robot.frontLeft.setPower(lfPower * 1.08);
                robot.frontRight.setPower(rfPower * 1.08);
                robot.backLeft.setPower(lrPower * 1.08);
                robot.backRight.setPower(rrPower * 1.08);


                angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                currentHeading = (360 + angles.firstAngle) % 360;
                if (currentHeading == lastAngle) {
                    if (robot.gameTimer.milliseconds() - lastTurnTime > 2000) {

                        robot.frontLeft.setPower(0);
                        robot.frontRight.setPower(0);
                        robot.backLeft.setPower(0);
                        robot.backRight.setPower(0);
                        state = MoveState.Finished;
                    }
                } else {
                    lastTurnTime = robot.gameTimer.milliseconds();
                    lastAngle = currentHeading;
                }
            }
            else {


                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
                state = MoveState.Finished;
            }
        }

    }
    public class CatWalk extends Move {
        double targetInchesY;
        double targetInchesX;

        double minPower = 0.1;
        double targetPower;
        double power = minPower;

        double powerX;

        double targetHeading;
        double aggresion;
        boolean done;
        double distanceTolerance;


        public CatWalk(Trigger triggerPARAM, double targetInchesYPARAM, double targetInchesXPARAM, double powerPARAM, double targetHeadingPARAM, double aggresionPARAM, double distancTolerancePARAM) {
            super(triggerPARAM, true);
            targetInchesY = targetInchesYPARAM;
            targetInchesX = targetInchesXPARAM;

            power = powerPARAM;
            powerX = Math.signum(targetInchesXPARAM);

            targetPower = powerPARAM;
            targetHeading = targetHeadingPARAM;
            aggresion = aggresionPARAM;

            distanceTolerance = distancTolerancePARAM;
        }

        public void Init() {
            state = MoveState.Init;
            ResetEncoders();

            double currentInches;
            double distanceToTarget;

            double currentStrafeInches;
            double strafeDistanceToTarget;

            double lfPower = power;
            double rfPower = power;
            double lrPower = power;
            double rrPower = power;

            double reverse;
            boolean strafeDone = false;

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInchesY - currentInches;

            if (Math.abs(distanceToTarget) > DISTANCE_TOLERANCE && opModeIsActive()) {

                double turnAdjustment;
                turnAdjustment = headingAdjustment(targetHeading, 0);

                currentStrafeInches = GetAverageStrafePositionInches();
                strafeDistanceToTarget = targetInchesX - currentStrafeInches;

                currentInches = GetAverageWheelPositionInches();
                distanceToTarget = targetInchesY - currentInches;

                if (distanceToTarget < 0) {
                    reverse = -1;
                } else {
                    reverse = 1;
                }
                double strafePower = (Math.pow(strafeDistanceToTarget / 10, 2) + 0.05) * powerX * aggresion;

                if (Math.abs(strafeDistanceToTarget) > 1) {
                    robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment + (strafePower));
                    robot.frontRight.setPower((rfPower * reverse) - turnAdjustment - (strafePower));
                    robot.backRight.setPower((rrPower * reverse) - turnAdjustment + (strafePower));
                    robot.backLeft.setPower((lrPower * reverse) + turnAdjustment - (strafePower));

                }
                else {

                    robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment);
                    robot.frontRight.setPower((rfPower * reverse) - turnAdjustment);
                    robot.backRight.setPower((rrPower * reverse) - turnAdjustment);
                    robot.backLeft.setPower((lrPower * reverse) + turnAdjustment);
                }
            }

        }
        public void Check() {

            double currentInches;
            double distanceToTarget;

            double currentStrafeInches;
            double strafeDistanceToTarget;

            double lfPower = power;
            double rfPower = power;
            double lrPower = power;
            double rrPower = power;

            double reverse;
            double headingAdjustmentMultiplier = 1.75;

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInchesY - currentInches;

            currentStrafeInches = GetAverageStrafePositionInches();
            strafeDistanceToTarget = targetInchesX - currentStrafeInches;

            if ((Math.abs(distanceToTarget) > distanceTolerance || (Math.abs(strafeDistanceToTarget) > 1.5 && !done)) && opModeIsActive()) {

                double turnAdjustment;
                turnAdjustment = headingAdjustment(targetHeading, 0);

                currentStrafeInches = GetAverageStrafePositionInches();
                strafeDistanceToTarget = targetInchesX - currentStrafeInches;

                currentInches = GetAverageWheelPositionInches();
                distanceToTarget = targetInchesY - currentInches;

                if (distanceToTarget < 0) {
                    reverse = -1;
                } else {
                    reverse = 1;
                }

                if (Math.abs(strafeDistanceToTarget) > 2 && !done) {
                    double strafePower = (Math.pow(strafeDistanceToTarget / 12, 2)  * powerX) + 0.05;
                    turnAdjustment = turnAdjustment * power * headingAdjustmentMultiplier;
                    robot.frontLeft.setPower((power * reverse) + turnAdjustment + (strafePower));
                    robot.frontRight.setPower((power * reverse) - turnAdjustment - (strafePower));
                    robot.backRight.setPower((power * reverse) - turnAdjustment + (strafePower));
                    robot.backLeft.setPower((power * reverse) + turnAdjustment - (strafePower));

                }
                if (Math.abs(strafeDistanceToTarget) <= 2) {
                    done = true;
                }
                if (done) {
                    telemetry.addData("Done", "true");
                    double powerCurve = power;


                    turnAdjustment = turnAdjustment * powerCurve * headingAdjustmentMultiplier;
                    robot.frontLeft.setPower((powerCurve * reverse) + turnAdjustment);
                    robot.frontRight.setPower((powerCurve * reverse) - turnAdjustment);
                    robot.backRight.setPower((powerCurve * reverse) - turnAdjustment);
                    robot.backLeft.setPower((powerCurve * reverse) + turnAdjustment);
                }
                telemetry.addData("distance", strafeDistanceToTarget);
            }
            else {
                telemetry.addData("strafeInchesPerTick", robot.strafeTicksPerInch);
                telemetry.addData("ticks", ((robot.frontLeft.getCurrentPosition() - robot.frontRight.getCurrentPosition() - robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4.0));
                telemetry.update();

                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
                state = MoveState.Finished;

            }

        }

    }

    public class MoveLift extends Move {
        double liftTarget;
        double liftPast;

        ElapsedTime moveTimer = new ElapsedTime();
        public MoveLift(Trigger triggerPARAM, double liftTargetPARAM, double liftPastPARAM) {
            super(triggerPARAM, false);
            liftTarget = liftTargetPARAM;
            liftPast = liftPastPARAM;
        }

        public void Init() {
            state = MoveState.Init;
            lift.SetPosition(liftTarget, liftPast, -1);
            state = MoveState.Finished;

        }


    }
    public class MoveDropper extends Move {
        String dropState;


        public MoveDropper(Trigger triggerPARAM, String statePARAM) {
            super(triggerPARAM, false);
            dropState = statePARAM;
        }

        public void Init() {
            state = MoveState.Init;
            if (dropState.equals("open")) {
                dropper.OpenDropper();
            }
            else {
                dropper.CloseDropper();
            }
            state = MoveState.Finished;

        }

    }

    public class Strafe extends Move {
        boolean strafeStarted = false;
        double targetStrafeInches;
        double power;
        int targetTag;
        double targetHeading;
        boolean tagDetected = false;
        double headingAdjustmentMultiplier = 2;
        public Strafe(Trigger triggerPARAM, double targetInchesPARAM, double powerPARAM, double headingPARAM, int tagPARAM) {
            super(triggerPARAM, true);
            targetStrafeInches = targetInchesPARAM;
            power = powerPARAM;
            targetHeading = headingPARAM;
            targetTag = tagPARAM;
        }
        public void Init() {
            state = MoveState.Init;
        }
        public void Check() {
            if (!strafeStarted) {
                ResetEncoders();
                if (targetTag != -1) {
                    lift.SetPosition(lift.liftAprilTags, 0, -1);
                }

                strafeStarted = true;

            }

            double currentStrafeInches =  GetAverageStrafePositionInches();
            double strafeDistanceToTarget = (targetStrafeInches * Math.signum(power)) - currentStrafeInches;



            if (Math.abs(strafeDistanceToTarget) > 0.5 && !tagDetected) {
                double turnAdjustment;
                turnAdjustment = headingAdjustment(targetHeading, 0);
                turnAdjustment = turnAdjustment; // * power * headingAdjustmentMultiplier;
                robot.frontLeft.setPower(power + turnAdjustment);
                robot.backLeft.setPower(-1 * (power) + turnAdjustment);
                robot.frontRight.setPower(-1 *(power) - turnAdjustment);
                robot.backRight.setPower(power - turnAdjustment);
                telemetry.addData("distance to target", strafeDistanceToTarget);
                AprilTagPoseFtc tagSeen = robot.getTargetAprilTagPos(targetTag);
                if (tagSeen != null) {
                    telemetry.addData("Target Tag Seen", tagSeen.x);

                    if (tagSeen.x < 2 && tagSeen.x > -2) {
                        telemetry.addData("Done", tagSeen.x);


                        robot.frontLeft.setPower(0);
                        robot.frontRight.setPower(0);
                        robot.backLeft.setPower(0);
                        robot.backRight.setPower(0);
                        telemetry.update();
                        tagDetected = true;


                    }


                }
                telemetry.update();
            }
            else {

                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
                state = MoveState.Finished;
            }


        }
    }

    //Old Auto Controls
    public void ManualTurn (double targetHeading) {

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;
        double currentHeading;
        ElapsedTime turnTimer = new ElapsedTime();
        double lastAngle = -1;
        while (degreesOff(targetHeading) > 0.5 && opModeIsActive()) {

            double adjustment = 0;
            adjustment = headingAdjustment(targetHeading, 0);

            lfPower = adjustment;
            rfPower = -adjustment;
            lrPower = adjustment;
            rrPower = -adjustment;

            robot.frontLeft.setPower(lfPower);
            robot.frontRight.setPower(rfPower);
            robot.backLeft.setPower(lrPower);
            robot.backRight.setPower(rrPower);


            angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            currentHeading = (360 + angles.firstAngle) % 360;
            if (currentHeading == lastAngle) {
                turnTimer.startTime();
                if (turnTimer.milliseconds() >= 2000) {

                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);
                    break;
                }
            }

            lastAngle = currentHeading;

        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

    }


    public void DriveWithCorrection (double targetInches, double targetHeading, double targetPower) {
        ResetEncoders();

        double currentInches;
        double distanceToTarget;



        double reverse;
        double headingAdjustmentMultiplier = 2.5;

        currentInches = GetAverageWheelPositionInches();
        distanceToTarget = targetInches - currentInches;

        double minPower = 0.3;

        double power = minPower;

        double distanceSensorTolerance = 6;

        while (Math.abs(distanceToTarget) > DISTANCE_TOLERANCE && opModeIsActive()) {

            if (Math.abs(targetInches) > 70 && Math.abs(distanceToTarget) / Math.abs(targetInches) < 0.25) {
                power -= 0.05;
                if (power < minPower) {
                    power = minPower;
                }
            }
            else {
                power += 0.05;
                if (power > targetPower) {
                    power = targetPower;
                }

            }

            while (robot.backLeftDS.getDistance(DistanceUnit.INCH) < distanceSensorTolerance && robot.backRightDS.getDistance(DistanceUnit.INCH) < distanceSensorTolerance && Math.abs(targetInches) > 70 && opModeIsActive()) {

                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

            }

            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = (360 + angles.firstAngle) % 360;

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            if (distanceToTarget < 0) {
                reverse = -1;
            } else {
                reverse = 1;
            }

            turnAdjustment = turnAdjustment * power * headingAdjustmentMultiplier;
            robot.frontLeft.setPower(power * reverse + turnAdjustment);
            robot.frontRight.setPower(power * reverse - turnAdjustment);
            robot.backRight.setPower(power * reverse - turnAdjustment);
            robot.backLeft.setPower(power * reverse + turnAdjustment);

            telemetry.addData("headingAdjustment: ", turnAdjustment);
            telemetry.addData("heading: ", currentHeading);
            telemetry.update();

            /*
            robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment);
            robot.frontRight.setPower((rfPower * reverse) - turnAdjustment);
            robot.backRight.setPower((rrPower * reverse) - turnAdjustment);
            robot.backLeft.setPower((lrPower * reverse) + turnAdjustment);
            */
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

    }

    public void DriveShortDistance (double targetInches, double power) {
        ResetEncoders();

        double currentInches;
        double distanceToTarget;
        double reverse;
        boolean targetReached = false; // actually means target passed

        currentInches = GetAverageWheelPositionInches();
        distanceToTarget = targetInches - currentInches;

        while (!targetReached && opModeIsActive()) {

            if (distanceToTarget < 0) {
                reverse = -1;
            } else {
                reverse = 1;
            }

            robot.frontLeft.setPower(power * reverse);
            robot.frontRight.setPower(power * reverse);
            robot.backRight.setPower(power * reverse);
            robot.backLeft.setPower(power * reverse);

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            if (targetInches < 0){     // This makes it so it can't oscillate.  No distance tolerance.  Even if it overshoots, it's done
                targetReached = (distanceToTarget > 0);
            } else {
                targetReached = (distanceToTarget < 0);
            }
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

    }

    public void DriveWithCorrectionDetectStop (double targetInches, double targetHeading, double power) {
        ResetEncoders();

        double currentInches;
        double distanceToTarget;
        double reverse;
        boolean stopped = false;
        boolean beginDetecting = false;
        double speedToBeginDetecting = .25;
        double stoppedSpeed = .05;
        boolean targetReached = false; // actually means target passed

        currentInches = GetAverageWheelPositionInches();
        distanceToTarget = targetInches - currentInches;

        while (!targetReached && !stopped && opModeIsActive()) {

            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            if (distanceToTarget < 0) {
                reverse = -1;
            } else {
                reverse = 1;
            }
            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            robot.frontLeft.setPower(power * reverse + turnAdjustment);
            robot.frontRight.setPower(power * reverse - turnAdjustment);
            robot.backRight.setPower(power * reverse - turnAdjustment);
            robot.backLeft.setPower(power * reverse + turnAdjustment);

            double currentLowestWheelVelocity = GetLowestWheelVelocity();

            if (currentLowestWheelVelocity > speedToBeginDetecting) {
                beginDetecting = true;
            }
            if (currentLowestWheelVelocity < stoppedSpeed && beginDetecting) {
                stopped = true;
            }
            if (targetInches < 0){   // This makes it so it can't oscillate.  No distance tolerance.  Even if it overshoots, it's done
                targetReached = (distanceToTarget >= 0);
            } else {
                targetReached = (distanceToTarget <= 0);
            }

            telemetry.addData("Current Velocity: ", currentLowestWheelVelocity);
            telemetry.addData("Stopped: ", stopped);
            telemetry.update();
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        DriveShortDistance(-0.6, 0.1);

    }

    public void DriveAtAngleToStack(double targetInches, double power) {
        ResetEncoders();

        //disableTagVision();

        double frameWidthInPixels = robot.STREAM_WIDTH;
        double cameraCenterAdjustPixels = -60;
        double frameCenterX = frameWidthInPixels / 2.0 + cameraCenterAdjustPixels;  // adjust this +/- some pixels to adjust off center camera
        double stackCenterX;
        double stackPercentFromCenter = 1;
        double stackNotFound = -1;

        double targetHeading;

        stackCenterX = robot.getCenter();

        while (stackCenterX == stackNotFound && opModeIsActive()) {
            stackCenterX = robot.getCenter();
        }


        double currentHeading = 0;

        for (int i = 0; i < 50; i++) {
            angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            currentHeading = (360 + angles.firstAngle) % 360;

            stackCenterX = robot.getCenter();

        }

        if (stackCenterX != stackNotFound) {
            stackPercentFromCenter = (stackCenterX - frameCenterX) / frameWidthInPixels;  // This returns percent of frame. if stack is to the left of center (lower X), negative strafePower should go left
        }

        double stackPercentMultiplied = stackPercentFromCenter * -50;

        /*
        if (Math.abs(stackPercentMultiplied) > 15) {
            telemetry.addData("adjustmentAngle", stackPercentMultiplied);
            telemetry.addData("stackPercentFromCenter", stackPercentFromCenter * -50);


            telemetry.update();

            stackPercentMultiplied = 0;
        }
        */

        targetHeading = currentHeading + stackPercentMultiplied;
        targetHeading = (targetHeading + 360) % 360;

        telemetry.addData("adjustmentAngle", stackPercentMultiplied);
        telemetry.addData("stackPercentFromCenter", stackPercentFromCenter * -50);
        telemetry.addData("angle", targetHeading);

        telemetry.update();

        //resumeAllVision();

        DriveWithCorrectionDetectStop(targetInches, targetHeading, power);
    }

    public double StrafeWithInchesWithCorrection(double targetStrafeInches, double power, int targetTag, int targetHeading) {
        ResetEncoders();

        //disableStackVision();

        double initialStrafeInches = GetAverageStrafePositionInches();
        double currentStrafeInches =  GetAverageStrafePositionInches();

        double inchesStrafed;

        double strafeDistanceToTarget = (targetStrafeInches * Math.signum(power)) - currentStrafeInches;


        double targetRange = 0;

        while (opModeIsActive() && Math.abs(strafeDistanceToTarget) > 0.5) {
            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            currentStrafeInches =  GetAverageStrafePositionInches();
            strafeDistanceToTarget = (targetStrafeInches * Math.signum(power)) - currentStrafeInches;

            robot.frontLeft.setPower(power + turnAdjustment);
            robot.backLeft.setPower(-1 * (power) + turnAdjustment);
            robot.frontRight.setPower(-1 *(power) - turnAdjustment);
            robot.backRight.setPower(power - turnAdjustment);
            //telemetry.addData("tag", robot.getFirstAprilTagID());
            AprilTagPoseFtc tagSeen = robot.getTargetAprilTagPos(targetTag);
            if (tagSeen != null) {
                telemetry.addData("Target Tag Seen", tagSeen.x);

                if (robot.alliance == "red") {
                    if (tagSeen.x > -1.5) {
                        //telemetry.addData("Done", tagSeen.x);
                        //targetRange = robot.getTargetAprilTagPos(targetTag).range;
                        inchesStrafed = -1;

                        robot.frontLeft.setPower(0);
                        robot.frontRight.setPower(0);
                        robot.backLeft.setPower(0);
                        robot.backRight.setPower(0);
                        telemetry.update();

                        //resumeAllVision();

                        return inchesStrafed;
                    }
                } else {
                    if (tagSeen.x < 1.5) {
                        //telemetry.addData("Done", tagSeen.x);
                        //targetRange = robot.getTargetAprilTagPos(targetTag).range;
                        inchesStrafed = -1;

                        robot.frontLeft.setPower(0);
                        robot.frontRight.setPower(0);
                        robot.backLeft.setPower(0);
                        robot.backRight.setPower(0);
                        telemetry.update();

                        //resumeAllVision();

                        return inchesStrafed;
                    }
                }


            }
            else {
                telemetry.addData("no tag", "");
            }


        }

        inchesStrafed = Math.abs(currentStrafeInches - initialStrafeInches) + 2;

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        //resumeAllVision();

        return inchesStrafed;
    }

    public double StrafeWithInchesWithCorrectionWithDistanceSensors(double targetStrafeInches, double power, int targetTag, int targetHeading) {
        ResetEncoders();

        //disableStackVision();

        double initialStrafeInches = GetAverageStrafePositionInches();
        double currentStrafeInches =  GetAverageStrafePositionInches();

        double inchesStrafed;

        double strafeDistanceToTarget = (targetStrafeInches * Math.signum(power)) - currentStrafeInches;


        double targetRange = 0;

        double distanceSensorTolerance = 6;

        double motorPower = power;

        while (opModeIsActive()/* && Math.abs(strafeDistanceToTarget) > 0.5*/) {
            motorPower = power;
            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            currentStrafeInches =  GetAverageStrafePositionInches();
            strafeDistanceToTarget = (targetStrafeInches * Math.signum(power)) - currentStrafeInches;

            if (robot.leftDS.getDistance(DistanceUnit.INCH) < distanceSensorTolerance || robot.rightDS.getDistance(DistanceUnit.INCH) < distanceSensorTolerance) {

                motorPower = 0;
                turnAdjustment = 0;
            }

            robot.frontLeft.setPower(motorPower + turnAdjustment);
            robot.backLeft.setPower(-1 * (motorPower) + turnAdjustment);
            robot.frontRight.setPower(-1 * (motorPower) - turnAdjustment);
            robot.backRight.setPower(motorPower - turnAdjustment);
            //telemetry.addData("tag", robot.getFirstAprilTagID());
            AprilTagPoseFtc tagSeen = robot.getTargetAprilTagPos(targetTag);



            if (tagSeen != null) {
                telemetry.addData("Target Tag Seen", tagSeen.x);

                if (robot.alliance == "red") {
                    if (tagSeen.x > -1.5) {
                        //telemetry.addData("Done", tagSeen.x);
                        //targetRange = robot.getTargetAprilTagPos(targetTag).range;
                        inchesStrafed = -1;

                        robot.frontLeft.setPower(0);
                        robot.frontRight.setPower(0);
                        robot.backLeft.setPower(0);
                        robot.backRight.setPower(0);
                        telemetry.update();

                        //resumeAllVision();

                        return inchesStrafed;
                    }
                } else {
                    if (tagSeen.x < 1.5) {
                        //telemetry.addData("Done", tagSeen.x);
                        //targetRange = robot.getTargetAprilTagPos(targetTag).range;
                        inchesStrafed = -1;

                        robot.frontLeft.setPower(0);
                        robot.frontRight.setPower(0);
                        robot.backLeft.setPower(0);
                        robot.backRight.setPower(0);
                        telemetry.update();

                        //resumeAllVision();

                        return inchesStrafed;
                    }
                }


            }
            else {
                telemetry.addData("no tag", "");
            }




        }

        inchesStrafed = Math.abs(currentStrafeInches - initialStrafeInches) + 2;

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        //resumeAllVision();

        return inchesStrafed;
    }


    /*public void disableStackVision() {
        robot.visionPortal.stopLiveView();

        robot.webcam.stopStreaming();
    }
    public void disableTagVision() {
        robot.visionPortal.stopLiveView();

        robot.visionPortal.stopStreaming();

    }

    public void resumeAllVision() {

        //robot.visionPortal.resumeStreaming();

        robot.webcam.startStreaming(robot.STREAM_WIDTH, robot.STREAM_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
    }*/



}