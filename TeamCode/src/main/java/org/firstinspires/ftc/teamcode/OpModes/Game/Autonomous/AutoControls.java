package org.firstinspires.ftc.teamcode.OpModes.Game.Autonomous;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public abstract class AutoControls extends LinearOpMode {

    boolean log = false;
    boolean doTelemetry = true;

    Robot robot;
    BNO055IMU imu;
    Orientation angles;
    OpenCvWebcam webcam;

    double countsPerInch = Robot.ticksPerInch;

    //lift
    static double liftSpeedUp = 1;
    double liftSpeedDown = .5;
    double liftSpeedPower;

    /*
    static double liftTicksPerInch = Robot.liftTicksPerInch;
    */

    //strafe per inch
    double strafeTicksPerInch = 100.0 / (7.0/4.0);

    static double multiplier = 0;
    static public char alliance = 'b';
    static public char side = 'l';


    static ElapsedTime gameTimer = new ElapsedTime();

    static double quitTime = 29850;

    public void init(HardwareMap hwMap) {
        Robot robot = new Robot(hwMap, false);
        initIMU();
        initCamera();
        telemetry.addData("here", "here");
        telemetry.update();

        multiplier = getVoltageMultiplier();
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

        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);  // puts it in config mode
        sleep(100);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

    }

    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

    }
    public double getVoltageMultiplier() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double multiplier = 1;
        if (voltage > 12) {
            multiplier = 1 + ((voltage - 13) / 13);
        }

        return multiplier;
    }

    public double getAverageOdometerPosition() {
        return ((robot.odometerLeft.getCurrentPosition() + robot.odometerRight.getCurrentPosition()) / 2.0) / robot.odometerTicksPerInch;
    }
    public double GetAverageVelocity() {
        double averageVelocity = 0;
        double left = robot.odometerLeft.getVelocity();
        double right =  robot.odometerRight.getVelocity();
        averageVelocity = (left + right) / 2;
        averageVelocity = (averageVelocity / robot.odometerTicksPerInch) / 12;
        return averageVelocity;
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

    public double GetTurningVelocity() {
        double turningVelocity = 0;
        double left = robot.odometerLeft.getVelocity();
        double right =  robot.odometerRight.getVelocity();
        turningVelocity = Math.abs(right - left) / 2;
        return turningVelocity;
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
        double speedModifier = 7;

        if (degreesOff > 10) {
            speedModifier = 10;
        }

        if (distanceToX == 0) {  // this????
            speedMinimum = 15;
        } else {
            speedMinimum = 3;
        }

        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + 2) / speedModifier, 2) + speedMinimum) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }
        if (log) {
            Log.d("headingAdjustment", "currentHeading:" + currentHeading +
                    " targetHeading:" + targetHeading +
                    " Adjustment:" + adjustment +
                    " DistanceToX:" + distanceToX +
                    " SpeedModifier:" + 7 +
                    " SpeedMinimum:" + 15 +
                    " Degrees Off:" + degreesOff);
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

        if (log) {
            Log.d("degreesOff", "degreesOff:" + degreesOff +
                    " currentHeading:" + currentHeading +
                    " targetHeading:" + targetHeading);
        }
        return degreesOff;
    }

    /*
    public void RaiseLift(double liftHeightTarget) {
        Robot.liftMotor.setTargetPosition((int) (liftHeightTarget * liftTicksPerInch));
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    */

    public void ResetEncoders() {
        robot.odometerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.odometerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void StopEncoders() {
        robot.odometerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

}