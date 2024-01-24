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

    public double DISTANCE_TOLERANCE = 0.15;
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
    // Assuming you have motors and encoders defined in your OpMode class


    // Function to move the robot forward and sideways




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
                telemetry.addData("time: ", gameTimer.milliseconds());
                telemetry.update();

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
                telemetry.addData("finished", totalFinished);
                telemetry.addData("size", moves.size());
                telemetry.update();
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

        double minPower = 0.1;
        double targetPower;
        double power = minPower;

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

            currentInchesCompare = currentInches;

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

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            currentInchesCompare = currentInches;
            telemetry.addData("compare", currentInchesCompare);
            telemetry.update();
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

            double lfPower = power;
            double rfPower = power;
            double lrPower = power;
            double rrPower = power;

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
        public Turn(Trigger triggerPARAM, double targetHeadingPARAM) {
            super(triggerPARAM, true);
            targetHeading = targetHeadingPARAM;
        }

        public void Init() {
            turnTimer.reset();
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

                lastAngle = currentHeading;

            }

        }
        public void Check() {
            turnTimer.reset();
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

        double minPower = 0.1;
        double targetPower;
        double power = minPower;

        double powerX;

        double targetHeading;
        double aggresion;
        boolean done;

        Vision vision;
        public CatWalk(Trigger triggerPARAM, double targetInchesYPARAM, double targetInchesXPARAM, double powerPARAM, double targetHeadingPARAM, double aggresionPARAM, Vision visionPARAM) {
            super(triggerPARAM, true);
            targetInchesY = targetInchesYPARAM;
            targetInchesX = targetInchesXPARAM;
            power = powerPARAM;
            powerX = Math.signum(targetInchesXPARAM);
            targetPower = powerPARAM;
            targetHeading = targetHeadingPARAM;
            aggresion = aggresionPARAM;
            vision = visionPARAM;
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

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInchesY - currentInches;

            currentInchesCompare = currentInches;

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

            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInchesY - currentInches;

            currentStrafeInches = GetAverageStrafePositionInches();
            strafeDistanceToTarget = targetInchesX - currentStrafeInches;

            currentInchesCompare = currentInches;

            if ((Math.abs(distanceToTarget) > DISTANCE_TOLERANCE || Math.abs(strafeDistanceToTarget) > 1) && opModeIsActive()) {

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

                if (Math.abs(strafeDistanceToTarget) > 1 && !done && !vision.CheckVision()) {
                    double strafePower = (Math.pow(strafeDistanceToTarget / 10, 2) + 0.05) * powerX;
                    robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment + (strafePower));
                    robot.frontRight.setPower((rfPower * reverse) - turnAdjustment - (strafePower));
                    robot.backRight.setPower((rrPower * reverse) - turnAdjustment + (strafePower));
                    robot.backLeft.setPower((lrPower * reverse) + turnAdjustment - (strafePower));
                }
                else {
                    done = true;

                }
                if (done) {
                    telemetry.addData("Done", "true");
                    double powerCurve = (Math.pow(distanceToTarget / 10, 2) + 0.05);
                    lfPower = powerCurve;
                    rfPower = powerCurve;
                    lrPower = powerCurve;
                    rrPower = powerCurve;
                    robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment);
                    robot.frontRight.setPower((rfPower * reverse) - turnAdjustment);
                    robot.backRight.setPower((rrPower * reverse) - turnAdjustment);
                    robot.backLeft.setPower((lrPower * reverse) + turnAdjustment);
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
            lift.SetPosition(liftTarget, liftPast);
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
            lift.SetPosition(lift.liftAprilTags, 0);
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
                telemetry.addData("adjustment", tagAdjustment);
                //telemetry.update();
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
        double cameraCenterAdjustPixels = 100;
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
                telemetry.addData("adjustment", strafePower);
                //telemetry.update();
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

            telemetry.addData("stackCenterX", stackCenterX);
            telemetry.addData("frameCenter", frameCenterX);
            telemetry.addData("frameWidthInPixels", frameWidthInPixels);
            telemetry.addData("strafePower", strafePower);
            telemetry.addData("stackPercentFromCenter", stackPercentFromCenter);
            telemetry.addData("distanceToTarget", distanceToTarget);


            telemetry.update();

        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public double StrafeWithInchesWithCorrection(double targetStrafeInches, double power, int targetTag, int targetHeading) {
        ResetEncoders();

        double currentStrafeInches =  (Math.abs(robot.frontLeft.getCurrentPosition()) + Math.abs(robot.frontRight.getCurrentPosition()) + Math.abs(robot.backLeft.getCurrentPosition()) + Math.abs(robot.backRight.getCurrentPosition()) / 4.0) / (100.0 / (7.0/4.0));

        double strafeDistanceToTarget = targetStrafeInches - currentStrafeInches;

        if (targetTag != -1) {
            lift.SetPosition(lift.liftAprilTags, 0);
        }
        double targetRange = 0;
        while (Math.abs(strafeDistanceToTarget) > 2 && opModeIsActive()) {
            double turnAdjustment;
            turnAdjustment = headingAdjustment(targetHeading, 0);

            currentStrafeInches = GetAverageStrafePositionInches();
            strafeDistanceToTarget = targetStrafeInches - currentStrafeInches;

            robot.frontLeft.setPower(power + turnAdjustment);
            robot.backLeft.setPower(-1 * (power) + turnAdjustment);
            robot.frontRight.setPower(-1 *(power) - turnAdjustment);
            robot.backRight.setPower(power - turnAdjustment);

            if (robot.getTargetAprilTagPos(targetTag) != null) {
                telemetry.addData("boolean", (robot.getTargetAprilTagPos(targetTag).x < 2 && robot.getTargetAprilTagPos(targetTag).x > -2));

                if (robot.getTargetAprilTagPos(targetTag).x < 2 && robot.getTargetAprilTagPos(targetTag).x > -2) {
                    telemetry.addData("target tag", robot.getTargetAprilTagPos(targetTag).x);

                    targetRange = robot.getTargetAprilTagPos(targetTag).range;
                    break;
                }
                telemetry.update();

            }
        }


        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        return targetRange;
    }




}