package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Dropper;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.IntakeHoist;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot.PixelSplitter;
import org.firstinspires.ftc.teamcode.Hardware.Robot.SpikeHook;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.List;

public abstract class AutoControlsCombined extends LinearOpMode {

    public Robot robot;
    public Dropper dropper;
    public Lift lift;
    public PixelSplitter splitter;
    public Intake intake;
    public SpikeHook spike;
    public IntakeHoist hoist;
    public BNO055IMU imu;
    public Orientation angles;

    public double DISTANCE_TOLERANCE = 0.5;
    public final double TAG_DISTANCE = 10;

    public double currentInchesCompare = -1;

    public ElapsedTime gameTimer = new ElapsedTime();


    public void initMethods(HardwareMap hwMap) {
        robot = new Robot(hwMap, false);
        dropper = robot.new Dropper();
        lift = robot.new Lift();
        intake = robot.new Intake();
        spike = robot.new SpikeHook();
        splitter = robot.new PixelSplitter();
        hoist = robot.new IntakeHoist();

        spike.ResetSpike();
        dropper.CloseDropper();
        splitter.OpenSplitter();

        robot.initAprilTag();
        robot.initEasyOpenCV();

        initIMU();
    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
    public void Strafe() {
        double power = 0.3;
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

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
        if (speedModifier == 0) {
            speedModifier = 5;
        }
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

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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


                int totalFinished = 0;

                for (Move move : moves) {
                    //init
                    if (move.trigger.CheckTrigger()) {
                        if (move.state == MoveState.Dead) {
                            move.state = MoveState.Init;
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

            }

        }
    }
    //VISION//
    public class Vision {

        public Vision() {

        }
        public boolean CheckVision() {
            return false;
        }
    }
    public class TagVision extends Vision {
        int targetTag;
        Motion drive;
        public TagVision(int targetTagPARAM) {
            targetTag = targetTagPARAM;
        }
        public boolean CheckVision() {
            boolean value = false;
            if (robot.getTargetAprilTagPos(targetTag) != null) {

                if (robot.getTargetAprilTagPos(targetTag).x < 1.5 && robot.getTargetAprilTagPos(targetTag).x > -1.5) {
                    value = true;

                }


            }

            return value;
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
    public class InchesTrigger extends Trigger {
        double inches;
        Motion drive;
        int ofDriveIndex;

        public InchesTrigger(double inchesPARAM, Motion drivePARAM, int ofDriveIndexPARAM) {
            inches = inchesPARAM;
            drive = drivePARAM;
            ofDriveIndex = ofDriveIndexPARAM;
        }
        public boolean CheckTrigger() {
            boolean value = Math.abs(inches) < Math.abs(currentInchesCompare) && drive.moves.get(ofDriveIndex).state == MoveState.Init;
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

        public void Check() {

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

        public void Check() {
            robot.intakeHoist.setPosition(hoistPosition);
            state = MoveState.Finished;
        }

    }

    public class MoveIntake extends Move {
        double moveIntakeForMilliseconds;

        ElapsedTime moveTimer = new ElapsedTime();
        boolean timerStarted = false;
        public MoveIntake(Trigger triggerPARAM, double moveIntakeForMillisecondsPARAM) {
            super(triggerPARAM, true);
            moveIntakeForMilliseconds = moveIntakeForMillisecondsPARAM;
        }

        public void Check() {
            intake.StartIntake(0.9);
            if (!timerStarted) {
                moveTimer.reset();
                timerStarted = true;
            }

            if (moveTimer.milliseconds() > moveIntakeForMilliseconds) {
                intake.StopIntake();
                state = MoveState.Finished;
            }
        }

    }

    public class Drive extends Move {
        double targetInches;

        double minPower = 0.1;
        double targetPower;
        double power = minPower;
        double headingAdjustmentMultiplier = 2;
        double targetHeading;
        boolean driveStarted = false;
        public Drive(Trigger triggerPARAM, double targetInchesPARAM, double powerPARAM, double targetHeadingPARAM) {
            super(triggerPARAM, true);
            targetInches = targetInchesPARAM;
            targetPower = powerPARAM;
            targetHeading = targetHeadingPARAM;
        }


        public void Check() {
            if (!driveStarted) {
                ResetEncoders();
                driveStarted = true;
            }

            double currentInches;
            double distanceToTarget;

            double reverse;

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            currentInchesCompare = currentInches;

            //acceleration
            if (Math.abs(distanceToTarget) / Math.abs(targetInches) < 0.15) {
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
        ElapsedTime turnTimer = new ElapsedTime();
        boolean turnStarted = false;
        public Turn(Trigger triggerPARAM, double targetHeadingPARAM) {
            super(triggerPARAM, true);
            targetHeading = targetHeadingPARAM;
        }


        public void Check() {
            if (!turnStarted) {
                turnTimer.reset();
                turnStarted = true;
                ResetEncoders();
            }
            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;
            double currentHeading;

            double lastAngle = -1;
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


                angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                currentHeading = (360 + angles.firstAngle) % 360;
                if (currentHeading == lastAngle) {
                    turnTimer.startTime();
                    if (turnTimer.milliseconds() >= 2000) {

                        robot.frontLeft.setPower(0);
                        robot.frontRight.setPower(0);
                        robot.backLeft.setPower(0);
                        robot.backRight.setPower(0);
                        state = MoveState.Finished;
                    }
                }

                lastAngle = currentHeading;

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



        double power;

        double powerX;

        double targetHeading;
        double aggresion;
        boolean done;
        double distanceTolerance;
        boolean catWalkStarted = false;
        Vision vision;
        public CatWalk(Trigger triggerPARAM, double targetInchesYPARAM, double targetInchesXPARAM, double powerPARAM, double targetHeadingPARAM, double aggresionPARAM, Vision visionPARAM, double distancTolerancePARAM) {
            super(triggerPARAM, true);
            targetInchesY = targetInchesYPARAM;
            targetInchesX = targetInchesXPARAM;

            power = powerPARAM;
            powerX = Math.signum(targetInchesXPARAM);

            targetHeading = targetHeadingPARAM;
            aggresion = aggresionPARAM;
            vision = visionPARAM;
            distanceTolerance = distancTolerancePARAM;
        }


        public void Check() {
            if (!catWalkStarted) {
                ResetEncoders();
                catWalkStarted = true;
            }
            double currentInches;
            double distanceToTarget;

            double currentStrafeInches;
            double strafeDistanceToTarget;

            double lfPower = power;
            double rfPower = power;
            double lrPower = power;
            double rrPower = power;

            double reverse;
            double headingAdjustmentMultiplier = 2;

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInchesY - currentInches;

            currentStrafeInches = GetAverageStrafePositionInches();
            strafeDistanceToTarget = targetInchesX - currentStrafeInches;

            currentInchesCompare = currentInches;

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

    public class MoveLift extends Move {
        double liftTarget;
        double liftPast;

        ElapsedTime moveTimer = new ElapsedTime();
        public MoveLift(Trigger triggerPARAM, double liftTargetPARAM, double liftPastPARAM) {
            super(triggerPARAM, false);
            liftTarget = liftTargetPARAM;
            liftPast = liftPastPARAM;
        }

        public void Check() {

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

        public void Check() {

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
                turnAdjustment = turnAdjustment * power * headingAdjustmentMultiplier;
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


            angles   = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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


    public void DriveWithCorrection (double targetInches, double targetHeading, double power) {
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

        while (Math.abs(distanceToTarget) > DISTANCE_TOLERANCE && opModeIsActive()) {

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

        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        DriveShortDistance(-0.3, 0.1);

    }

    public void DriveWithCorrectionToAprilTag(double targetInches, double targetHeading, double power, int targetTag) {
        ResetEncoders();

        AprilTagPoseFtc fetchedPose = null;

        double currentInches;
        double distanceToTarget;

        double lfPower = power;
        double rfPower = power;
        double lrPower = power;
        double rrPower = power;

        double reverse;

        currentInches = GetAverageWheelPositionInches();
        distanceToTarget = targetInches - currentInches;

        if (targetTag != -1) {
            lift.SetPosition(lift.liftAprilTags, 0, -1);
        }

        while (Math.abs(distanceToTarget) > DISTANCE_TOLERANCE && (fetchedPose == null ? true : fetchedPose.range > TAG_DISTANCE) && opModeIsActive()) {

            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            double tagAdjustment = 0;
            if (targetTag != -1) {
                fetchedPose = robot.getTargetAprilTagPos(targetTag);
                if (fetchedPose != null) {
                    //tagAdjustment = ((fetchedPose.x) / Math.abs(fetchedPose.x));
                    tagAdjustment = (0.2/5) * (fetchedPose.x);
                }
            }

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            if (distanceToTarget < 0) {
                reverse = -1;
            } else {
                reverse = 1;
            }

            robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment + tagAdjustment);
            robot.frontRight.setPower((rfPower * reverse) - turnAdjustment - tagAdjustment);
            robot.backRight.setPower((rrPower * reverse) - turnAdjustment + tagAdjustment);
            robot.backLeft.setPower((lrPower * reverse) + turnAdjustment - tagAdjustment);
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


    }

    public void DriveWithCorrectionToStack(double targetInches, double targetHeading, double power, double hoistPosition) {
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


        double tagAdjustment = -1;
        while (Math.abs(tagAdjustment) > 0.03 && Math.abs(distanceToTarget) > DISTANCE_TOLERANCE && opModeIsActive()) {

            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            tagAdjustment = 0;
            double left;
            if (robot.getCenter() != -1) {
                tagAdjustment = (((robot.getCenter() + (1280/34.0)) / 1280.0) - 0.5) * (1/3.0);

            }
            if (tagAdjustment < 0) {
                left = -1;
            }
            else {
                left = 1;
            }

            /*if (tagAdjustment < 0.01) {
                break;
            }*/


            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            if (distanceToTarget < 0) {
                reverse = -1;
            } else {
                reverse = 1;
            }

            robot.frontLeft.setPower((0) + turnAdjustment + (tagAdjustment + (0.1 * left)));
            robot.frontRight.setPower((0) - turnAdjustment - (tagAdjustment + (0.1 * left)));
            robot.backRight.setPower((0) - turnAdjustment + (tagAdjustment + (0.1 * left)));
            robot.backLeft.setPower((0) + turnAdjustment - (tagAdjustment + (0.1 * left)));

        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        if (tagAdjustment < 0.03) {
            sleep(100);
            robot.intakeHoist.setPosition(hoistPosition);
            //hoist.Stack();
        }


    }

    public void DriveWithCorrectionToStackTwo(double targetInches, double targetHeading, double power) {
        ResetEncoders();

        double currentInches;
        double distanceToTarget;
        double frameWidthInPixels = robot.STREAM_WIDTH;
        double cameraCenterAdjustPixels = 0;
        double frameCenterX = frameWidthInPixels / 2.0 + cameraCenterAdjustPixels;  // adjust this +/- some pixels to adjust off center camera
        double strafePower;
        double strafeAgressiveness = 0.5;
        double strafeSpeedMinimum = .08;
        double forwardSpeedMinimum = .06;
        double forwardPower;
        double speedModifier = 6;
        double stackCenterX;
        double stackPercentFromCenter = 1;
        double stackNotFound = -1;
        double stackCenterXTolerance = .03;
        boolean strafeDone = false;
        boolean startCollisionDetection = false;
        boolean collidedWithWall = false;
        double collidedVelocity = 0.03;

        currentInches = GetAverageWheelPositionInches();
        distanceToTarget = targetInches - currentInches;

        stackCenterX = robot.getCenter();

        if (stackCenterX != stackNotFound) {
            stackPercentFromCenter = (stackCenterX - frameCenterX) / frameWidthInPixels;  // This returns percent of frame. if stack is to the left of center (lower X), negative strafePower should go left
        }

        while (!strafeDone || Math.abs(distanceToTarget) > DISTANCE_TOLERANCE && opModeIsActive()) {

            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            if (stackCenterX != stackNotFound && Math.abs(stackPercentFromCenter) > stackCenterXTolerance && !strafeDone) {
                strafePower = Math.abs(stackPercentFromCenter) * strafeAgressiveness + strafeSpeedMinimum; // Example: 10% of frame could be like 2 inches.  So .1 to go 2 inches might be too much
                if (stackPercentFromCenter < 0) {
                    strafePower = strafePower * -1;
                }

                // Here we should recalculate distance to target using the stack width and proper pixels per inch
            } else {
                strafePower = 0;
                if (stackCenterX != stackNotFound) {
                    strafeDone = true;
                }
            }

            if (Math.abs(distanceToTarget) > DISTANCE_TOLERANCE) {
                forwardPower = power;
                if (distanceToTarget < 0) {
                    forwardPower = forwardPower * -1;
                }
            } else {
                forwardPower = 0;
            }

            robot.frontLeft.setPower(forwardPower + turnAdjustment + strafePower);
            robot.frontRight.setPower(forwardPower - turnAdjustment - strafePower);
            robot.backRight.setPower(forwardPower - turnAdjustment + strafePower);
            robot.backLeft.setPower(forwardPower + turnAdjustment - strafePower);

            stackCenterX = robot.getCenter();

            if (stackCenterX != stackNotFound) {
                stackPercentFromCenter = (stackCenterX - frameCenterX) / frameWidthInPixels;  // This returns percent of frame. if stack is to the left of center (lower X), negative strafePower should go left
            }

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;
            if (GetAverageVelocity() > collidedVelocity) {
                startCollisionDetection = true;
            }
            if (startCollisionDetection && GetAverageVelocity() < collidedVelocity) {
                collidedWithWall = true;
            }

            /*telemetry.addData("stackCenterX", stackCenterX);
            telemetry.addData("frameCenter", frameCenterX);
            telemetry.addData("frameWidthInPixels", frameWidthInPixels);
            telemetry.addData("strafePower", strafePower);
            telemetry.addData("stackPercentFromCenter", stackPercentFromCenter);
            telemetry.addData("distanceToTarget", distanceToTarget);*/


        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }
    public void DriveAtAngleToStack(double targetInches, double power) {
        ResetEncoders();

        double frameWidthInPixels = robot.STREAM_WIDTH;
        double cameraCenterAdjustPixels = -60;
        double frameCenterX = frameWidthInPixels / 2.0 + cameraCenterAdjustPixels;  // adjust this +/- some pixels to adjust off center camera
        double stackCenterX;
        double stackPercentFromCenter = 1;
        double stackNotFound = -1;

        double targetHeading;

       /* stackCenterX = robot.getCenter();

        if (stackCenterX != stackNotFound) {
            stackPercentFromCenter = (stackCenterX - frameCenterX) / frameWidthInPixels;  // This returns percent of frame. if stack is to the left of center (lower X), negative strafePower should go left
        }

        double stackPercentMultiplied = stackPercentFromCenter * 50;*/


        /*if (stackPercentMultiplied < 0) {
            targetHeading = 360 + stackPercentMultiplied;
        }
        else {
            targetHeading = stackPercentMultiplied;
        }*/
        /*

        while (stackCenterX == stackNotFound) {
           stackCenterX = robot.getCenter();

        }
        while (opModeIsActive()) {
            stackCenterX = robot.getCenter();

            if (stackCenterX != stackNotFound) {
                stackPercentFromCenter = (stackCenterX - frameCenterX) / frameWidthInPixels;  // This returns percent of frame. if stack is to the left of center (lower X), negative strafePower should go left
            }

            targetHeading = stackPercentFromCenter * 50;


            telemetry.addData("targetHeading", targetHeading);
            telemetry.update();
            //sleep(5000);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double currentHeading = (360 + angles.firstAngle) % 360;


            targetHeading = currentHeading + targetHeading;
            targetHeading = (targetHeading + 360) % 360;

            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("XLocation", robot.getCenter());
            telemetry.addData("area", robot.area);
            telemetry.addData("stackPercentFromCenter", stackPercentFromCenter);
            telemetry.addData("stackCenterX", stackCenterX);
            telemetry.addData("targetHeadingWithAdjustedAngle", targetHeading);
            telemetry.update();
            sleep(1000);
        }
        //DriveWithCorrectionDetectStop(targetInches, targetHeading, power);

        //DriveWithCorrection(targetInches, targetHeading, power);*/
        stackCenterX = robot.getCenter();

        while (stackCenterX == stackNotFound) {
            stackCenterX = robot.getCenter();
        }


        double currentHeading = 0;

        for (int i = 0; i < 50; i++) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            currentHeading = (360 + angles.firstAngle) % 360;

            stackCenterX = robot.getCenter();

        }

        if (stackCenterX != stackNotFound) {
            stackPercentFromCenter = (stackCenterX - frameCenterX) / frameWidthInPixels;  // This returns percent of frame. if stack is to the left of center (lower X), negative strafePower should go left
        }

        double stackPercentMultiplied = stackPercentFromCenter * -50;

        targetHeading = currentHeading + stackPercentMultiplied;
        targetHeading = (targetHeading + 360) % 360;


        DriveWithCorrectionDetectStop(targetInches, targetHeading, power);
    }

    public double StrafeWithInchesWithCorrection(double targetStrafeInches, double power, int targetTag, int targetHeading) {
        ResetEncoders();

        double initialInches = GetAverageStrafePositionInches();
        double inchesStrafed;

        double currentStrafeInches =  GetAverageStrafePositionInches();
        double strafeDistanceToTarget = (targetStrafeInches * Math.signum(power)) - currentStrafeInches;

        if (targetTag != -1) {
            lift.SetPosition(lift.liftAprilTags, 0, -1);
        }
        double targetRange = 0;

        while (Math.abs(strafeDistanceToTarget) > 0.5 && opModeIsActive()) {
            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            currentStrafeInches =  GetAverageStrafePositionInches();
            strafeDistanceToTarget = (targetStrafeInches * Math.signum(power)) - currentStrafeInches;

            robot.frontLeft.setPower(power + turnAdjustment);
            robot.backLeft.setPower(-1 * (power) + turnAdjustment);
            robot.frontRight.setPower(-1 *(power) - turnAdjustment);
            robot.backRight.setPower(power - turnAdjustment);
            telemetry.addData("distance to target", strafeDistanceToTarget);
            AprilTagPoseFtc tagSeen = robot.getTargetAprilTagPos(targetTag);
            if (tagSeen != null) {
                telemetry.addData("Target Tag Seen", tagSeen.x);

                if (tagSeen.x < 2 && tagSeen.x > -2) {
                    //telemetry.addData("Done", tagSeen.x);
                    targetRange = robot.getTargetAprilTagPos(targetTag).range;

                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);

                    inchesStrafed = GetAverageStrafePositionInches() - initialInches;

                    telemetry.addData("Inches Strafed: ", inchesStrafed);
                    telemetry.update();
                    sleep(500);

                    return inchesStrafed;
                }


            }
            telemetry.update();
        }


        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        inchesStrafed = GetAverageStrafePositionInches() - initialInches;

        //return targetRange;
        return inchesStrafed;
    }




}