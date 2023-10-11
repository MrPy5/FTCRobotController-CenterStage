package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;


public class Robot {

    //---Constants---//
    public static double triggerSensitivity = 0.01;

    //---INTAKE---//
    public static DcMotor intakeMotor;
    public static boolean intakeReset = true;

    public static int intakeState = 0;

    //---LIFT---//
    public static DcMotor liftMotor;

    //---DRONE LAUNCHER---//
    public static Servo droneReleaser;

    //---DROPPER---//
    public static Servo leftDropper;
    public static Servo rightDropper;

    //---DRIVING---//

    //Motors
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;

    public static double wheelCountsPerRevolution = 537.6;
    public static double wheelDiameter = 3.77;
    public static double ticksPerInch = (wheelCountsPerRevolution) /
            (wheelDiameter * Math.PI);

    public static double deadStickZone = 0.01;
    public static double wheelPowerMinToMove = 0.05;

    //Odometer
    public static DcMotorEx odometerLeft;
    public static DcMotorEx odometerRight;
    public static double odometerCountsPerRevolution = 8192;
    public static double odometerWheelDiameter = 1.436;
    public static double odometerTicksPerInch = (odometerCountsPerRevolution) /
            (odometerWheelDiameter * Math.PI);
    public static double workingEncoderVelocityDifference = 4;

    //Hardware Map
    public static HardwareMap hardwareMap;


    //---APRIL TAGS---//
    public static AprilTagProcessor aprilTag;

    public static VisionPortal visionPortal;

    public static boolean showCameraPreview = true;

    public String webCamName = "Webcam 1";

    //---EASY OPEN CV---//

    public static OpenCvWebcam webcam;

    public static ColorPipeline pipeline;

    static int STREAM_WIDTH = 1280; // modify for your camera
    static int STREAM_HEIGHT = 720; // modify for your camera

    //http://192.168.43.1:8080/dash


    public Robot(HardwareMap robot_hardwareMap, boolean show_CameraPreview) {
        //Set Hardware map
        hardwareMap = robot_hardwareMap;
        showCameraPreview = show_CameraPreview;


        //---Driving---//

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");


        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //---Odometer---//
        /*
        odometerLeft = hardwareMap.get(DcMotorEx.class, "odometerLeft");
        odometerRight = hardwareMap.get(DcMotorEx.class, "odometerRight");

        odometerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odometerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odometerRight.setDirection(DcMotorSimple.Direction.REVERSE);

        odometerLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        */
    }
    public void initEasyOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamNamed = null;
        webcamNamed = hardwareMap.get(WebcamName.class, webCamName); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamNamed, cameraMonitorViewId);
        pipeline = new ColorPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public void initAprilTag() {


        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();



        builder.setCamera(hardwareMap.get(WebcamName.class, webCamName));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(showCameraPreview);


        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();


    }
    public int getFirstAprilTagID() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        // Step through the list of detections and display info for each one.
        int id = -1;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                id = detection.id;

            }

            break;

        }   // end for() loop

        return id;


    }

    public double[] navigateToAprilTag(int targetId, int targetDistance) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        // Step through the list of detections and display info for each one.

        AprilTagDetection currentDetection = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                if (detection.id == targetId) {
                    currentDetection = detection;
                    break;
                }
            }

        }   // end for() loop
        if (currentDetection != null) {
            double distance = currentDetection.ftcPose.range;
            double x = currentDetection.ftcPose.x;
            double direction = (x / Math.abs(x));
            double distanceFromTarget = Math.min(Math.abs(x), 12) / 12;
            double driveDistanceFromTarget = Math.max(targetDistance, Math.min(Math.abs(distanceFromTarget), 60)) / 60;
            double turnPower = Math.sqrt(distanceFromTarget) * direction;
            double drivePower = Math.sqrt(driveDistanceFromTarget);

            return new double[]{turnPower,drivePower};
        }
        else {
            return new double[]{0,0};
        }
    }



    public static class Intake {
        public Intake() {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void StartIntake() {
            intakeMotor.setPower(0.3);
            intakeState = 1;
        }
        public void StopIntake() {
            intakeMotor.setPower(0);
            intakeState = 0;
        }
    }
    public static class Lift {

        public int currentLevel = 1;

        public static double liftMotorTicksPerRevolution = 537;
        public static double liftSpoolDiameter = 1;
        public static double liftCascadeMultiplier = 3;
        public static double liftTicksPerInch = liftMotorTicksPerRevolution / (liftSpoolDiameter * Math.PI * liftCascadeMultiplier);

        public static double liftPower = 0.3;

        public Lift() {
            liftMotor = hardwareMap.get(DcMotor.class, "lift");
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        public void SetPosition(double liftTargetPosition) {
            liftMotor.setPower(liftPower);
            liftMotor.setTargetPosition((int) (liftTargetPosition * liftTicksPerInch));
            currentLevel = 0; //edit this later;
        }
    }

    public static class Dropper {

        public boolean leftOpen = false;
        public String leftColor = "None";
        public double openLeft = 0;
        public double closedLeft = 0;

        public boolean rightOpen = false;
        public String rightColor = "None";
        public double openRight = 0;
        public double closedRight = 0;

        public Dropper() {
            leftDropper = hardwareMap.get(Servo.class, "leftDropper");
            rightDropper = hardwareMap.get(Servo.class, "rightDropper");
        }

        public void ClosedLeftDropper() {
            leftDropper.setPosition(closedLeft);
            leftOpen = false;
        }
        public void OpenLeftDropper() {
            leftDropper.setPosition(openLeft);
            leftOpen = true;
        }

        public void ClosedRightDropper() {
            rightDropper.setPosition(closedRight);
            rightOpen = false;
        }
        public void OpenRightDropper() {
            rightDropper.setPosition(openRight);
            rightOpen = true;
        }
    }

    public static class DroneLauncher {
        public double setPosition = 0;
        public double releasedPosition = 0;
        public DroneLauncher() {
            droneReleaser = hardwareMap.get(Servo.class, "droneReleaser");
        }

        public void ReleaseDrone() {
            droneReleaser.setPosition(releasedPosition);
        }

        public void Reset() {
            droneReleaser.setPosition(setPosition);
        }
    }

   public static class ColorPipeline extends OpenCvPipeline {

        Mat YCrCb = new Mat();
        Mat Y = new Mat();
        public static int avg;


        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Y channel to the 'Y' variable
         */
        void inputToY(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
            Core.split(YCrCb, yCrCbChannels);
            Y = yCrCbChannels.get(0);

        }

        @Override
        public void init(Mat firstFrame) {
            inputToY(firstFrame);
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToY(input);

            avg = (int) Core.mean(Y).val[0];
            YCrCb.release(); // don't leak memory!
            Y.release(); // don't leak memory!
            return input;
        }
        public static double GetOutput() {
            return avg;
        }

    }
}