package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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



        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


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
        public boolean rightOpen = false;

        public String leftColor = "None";
        public String rightColor = "None";
        public Dropper() {

        }

        public void CloseLeftGrabber() {
            leftOpen = false;
        }
        public void OpenLeftGrabber() {
            leftOpen = true;
        }

        public void CloseRightGrabber() {
            rightOpen = false;
        }
        public void OpenRightGrabber() {
            rightOpen = true;
        }
    }
}