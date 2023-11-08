package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.Hardware.Robot.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Intake;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class FirstMeetAutoControls extends LinearOpMode {

    Robot robot;
    Dropper dropper;
    Lift lift;

    Intake intake;
    BNO055IMU imu;
    Orientation angles;

    public void initMethods(HardwareMap hwMap) {
        robot = new Robot(hwMap, false);
        dropper = robot.new Dropper();
        lift = robot.new Lift();
        intake = robot.new Intake();
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

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /*byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);  // puts it in config mode
        sleep(100);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);*/

    }

    public double GetAverageVelocityMecanum() {
        double averageVelocity;

        double backLeft = robot.backLeft.getVelocity();
        double backRight = robot.backRight.getVelocity();
        double frontRight = robot.frontRight.getVelocity();
        double frontLeft = robot.frontLeft.getVelocity();

        averageVelocity = (backLeft + backRight + frontRight + frontLeft) / 4;
        averageVelocity = (averageVelocity / robot.ticksPerInch) / 12;
        return  averageVelocity;
    }

    public double GetAverageWheelPositionInches() {
        return ((((robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4.0) / robot.ticksPerInch));
    }

    public double GetAverageStrafePositionInches() {
        return (((Math.abs(robot.frontLeft.getCurrentPosition()) + Math.abs(robot.frontRight.getCurrentPosition()) + Math.abs(robot.backLeft.getCurrentPosition()) + Math.abs(robot.backRight.getCurrentPosition())) / 4.0) / robot.strafeTicksPerInch);
    }

    public double headingAdjustment(double targetHeading, double distanceToX) {
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
        double speedModifier = 6;

        if (degreesOff > 10) {
            speedModifier = 8;
        }

        speedMinimum = 3;


        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + 2) / speedModifier, 2) + speedMinimum) / 100;
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

        angles   = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        currentHeading = (360 + angles.firstAngle) % 360;

        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {

            degreesOff = 360 - degreesOff;
        }

        return degreesOff;
    }

    public void ZeroPowerToBrake() {
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void ZeroPowerToFloat() {
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    public void Turn (double targetHeading) {

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;
        double currentHeading;
        ElapsedTime turnTimer = new ElapsedTime();
        double lastAngle = -1;
        while (degreesOff(targetHeading) > 0.5) {

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
                if (turnTimer.milliseconds() >= 750) {

                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);
                    break;
                }
            }

            lastAngle = currentHeading;

        }

    }

    public void Drive (double targetInches) {
        ResetEncoders();
        double currentInches;
        double distanceToTarget;

        double lfPower = 0.5;
        double rfPower = 0.5;
        double lrPower = 0.5;
        double rrPower = 0.5;

        double reverse = 1;

        currentInches = GetAverageWheelPositionInches();
        distanceToTarget = targetInches - currentInches;

        if (distanceToTarget < 0) {
            reverse = -1;
        } else {
            reverse = 1;
        }

        while (Math.abs(distanceToTarget) > 1) {
            currentInches = GetAverageWheelPositionInches();
            distanceToTarget = targetInches - currentInches;

            if (distanceToTarget < 0) {
                reverse = -1;
            } else {
                reverse = 1;
            }
            robot.frontLeft.setPower(lfPower * reverse);
            robot.frontRight.setPower(rfPower * reverse);
            robot.backRight.setPower(rrPower * reverse);
            robot.backLeft.setPower(lrPower * reverse);
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


    }

    public void Strafe(long milliseconds, int direction) {
        if (direction == 1) {
            robot.frontLeft.setPower(0.2);
            robot.backLeft.setPower(-0.2);
            robot.frontRight.setPower(-0.2);
            robot.backRight.setPower(0.2);
        }
        if (direction == 0) {
            robot.frontLeft.setPower(-0.2);
            robot.backLeft.setPower(0.2);
            robot.frontRight.setPower(0.2);
            robot.backRight.setPower(-0.2);
        }
        double startTime = robot.gameTimer.milliseconds();
        while ((robot.gameTimer.milliseconds() < startTime + milliseconds)) {

        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);


    }

    public void StrafeWithInches(double targetStrafeInches, int direction) {
        ResetEncoders();

        double currentStrafeInches = GetAverageStrafePositionInches();
        double strafeDistanceToTarget = targetStrafeInches - currentStrafeInches;

        while (Math.abs(strafeDistanceToTarget) > 1) {
            currentStrafeInches = GetAverageStrafePositionInches();
            strafeDistanceToTarget = targetStrafeInches - currentStrafeInches;

            if (direction == 1) {
                robot.frontLeft.setPower(0.2);
                robot.backLeft.setPower(-0.2);
                robot.frontRight.setPower(-0.2);
                robot.backRight.setPower(0.2);
            }
            if (direction == 0) {
                robot.frontLeft.setPower(-0.2);
                robot.backLeft.setPower(0.2);
                robot.frontRight.setPower(0.2);
                robot.backRight.setPower(-0.2);
            }
        }


        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

}