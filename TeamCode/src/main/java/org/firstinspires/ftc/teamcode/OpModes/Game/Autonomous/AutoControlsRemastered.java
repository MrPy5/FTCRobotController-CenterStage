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

import java.util.ArrayList;
import java.util.List;

public abstract class AutoControlsRemastered extends LinearOpMode {

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
        return (((Math.abs(robot.frontLeft.getCurrentPosition()) + Math.abs(robot.frontRight.getCurrentPosition()) + Math.abs(robot.backLeft.getCurrentPosition()) + Math.abs(robot.backRight.getCurrentPosition())) / 4.0) / robot.strafeTicksPerInch);
    }
    public enum MoveState {
        Dead,
        Init,
        Finished,

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


    public class Motion {
        List<Move> moves = new ArrayList<>();

        public Motion() {

        }
        public void add(Move move) {
            moves.add(move);
        }
        public void Start(int millisecondDelay) {
            sleep(millisecondDelay);

            while (opModeIsActive()) {
                telemetry.addData("time: ", gameTimer.milliseconds());
                telemetry.update();

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
                }
            }

        }
    }
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
            intake.StartIntake(0.75);
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
            if (degreesOff(targetHeading) > 0.5 && opModeIsActive()) {

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
            if (degreesOff(targetHeading) > 0.5 && opModeIsActive()) {

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
        public CatWalk(Trigger triggerPARAM, double targetInchesYPARAM, double targetInchesXPARAM, double powerPARAM, double targetHeadingPARAM, double aggresionPARAM) {
            super(triggerPARAM, true);
            targetInchesY = targetInchesYPARAM;
            targetInchesX = Math.abs(targetInchesXPARAM);
            powerX = Math.signum(targetInchesXPARAM);
            targetPower = powerPARAM;
            targetHeading = targetHeadingPARAM;
            aggresion = aggresionPARAM;
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

                if (Math.abs(strafeDistanceToTarget) > 1) {
                    robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment + (powerX * aggresion));
                    robot.frontRight.setPower((rfPower * reverse) - turnAdjustment - (powerX * aggresion));
                    robot.backRight.setPower((rrPower * reverse) - turnAdjustment + (powerX * aggresion));
                    robot.backLeft.setPower((lrPower * reverse) + turnAdjustment - (powerX * aggresion));
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

                if (Math.abs(strafeDistanceToTarget) > 1) {
                    robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment + (powerX * aggresion));
                    robot.frontRight.setPower((rfPower * reverse) - turnAdjustment - (powerX * aggresion));
                    robot.backRight.setPower((rrPower * reverse) - turnAdjustment + (powerX * aggresion));
                    robot.backLeft.setPower((lrPower * reverse) + turnAdjustment - (powerX * aggresion));
                }
                else {
                    robot.frontLeft.setPower((lfPower * reverse) + turnAdjustment);
                    robot.frontRight.setPower((rfPower * reverse) - turnAdjustment);
                    robot.backRight.setPower((rrPower * reverse) - turnAdjustment);
                    robot.backLeft.setPower((lrPower * reverse) + turnAdjustment);
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





}