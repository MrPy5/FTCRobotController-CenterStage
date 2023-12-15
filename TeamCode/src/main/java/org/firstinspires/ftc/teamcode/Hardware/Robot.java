package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.*;

import org.opencv.imgproc.Moments;


import java.util.ArrayList;
import java.util.List;


public class Robot {

    //---Constants---//
    public double triggerSensitivity = 0.01;

    //---INTAKE---//
    public DcMotor intakeMotor;

    //---LIFT---//
    public DcMotor liftMotor;


    //---DRONE LAUNCHER---//
    public Servo droneReleaser;

    //---PIXEL SPLITTER--//
    public Servo pixelSplitter;

    public enum pixelSplitterState {
        Open,
        Split
    }

    //---SUSPENSION---//
    public DcMotor suspensionMotor;
    public Servo suspensionServo;

    //---DROPPER---//
    public Servo pixelDropper; //Delete if two droppers

    //---SPIKE HOOK---//
    public Servo spikeHook; //Delete if two droppers

    //---DRIVING---//

    //Motors
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;

    public double wheelCountsPerRevolution = 537.6;
    public double wheelDiameter = 3.77;
    public double ticksPerInch = (wheelCountsPerRevolution) /
            (wheelDiameter * Math.PI);

    public double deadStickZone = 0.01;
    public double wheelPowerMinToMove = 0.05;

    public double strafeTicksPerInch = 100.0 / (7.0/4.0);

    //Odometer
    public DcMotorEx odometerLeft;
    public DcMotorEx odometerRight;
    public double odometerCountsPerRevolution = 8192;
    public double odometerWheelDiameter = 1.436;
    public double odometerTicksPerInch = (odometerCountsPerRevolution) /
            (odometerWheelDiameter * Math.PI);
    public double workingEncoderVelocityDifference = 4;

    //Hardware Map
    public HardwareMap hardwareMap;


    //---APRIL TAGS---//
    public AprilTagProcessor aprilTag;

    public VisionPortal visionPortal;

    public boolean showCameraPreview = true;

    public String aprilTagWebCamName = "AprilTag";


    //---EASY OPEN CV---//

    public OpenCvWebcam webcam;

    public String openCVWebCamName = "OpenCV";

    public ColorCounter pipeline;

    public int STREAM_WIDTH = 1280; // modify for your camera
    public int STREAM_HEIGHT = 720; // modify for your camera

    public final double objectWidthInRealWorldUnits = 5.5;  // Replace with the actual width of the object in real-world units
    public final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    public String alliance = "";

    public double centerX = 0;
    public double centerY = 0;

    public int whiteOne = 0;
    public int whiteTwo = 0;

    public double area = 0;

    public ElapsedTime gameTimer = new ElapsedTime();


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

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





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
        webcamNamed = hardwareMap.get(WebcamName.class, openCVWebCamName); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamNamed, cameraMonitorViewId);
        //pipeline = new ContourDetectionPipeline();
        pipeline = new ColorCounter();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
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
                .setLensIntrinsics(912.854, 912.854, 621.648, 372.268)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();



        builder.setCamera(hardwareMap.get(WebcamName.class, aprilTagWebCamName));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new android.util.Size(1280, 800));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidtmh than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

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

    public AprilTagPoseFtc getFirstAprilTagPose() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        // Step through the list of detections and display info for each one.
        AprilTagPoseFtc pose = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                pose = detection.ftcPose;

            }

            break;

        }   // end for() loop

        return pose;


    }

    public AprilTagPoseFtc getTargetAprilTagPos(int targetID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        // Step through the list of detections and display info for each one.
        AprilTagPoseFtc pose = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == targetID) {

                    pose = detection.ftcPose;
                }

            }


        }   // end for() loop

        return pose;


    }

    public class Intake {
        public int intakeState = 0;
        public Intake() {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void StartIntake(double power) {
            intakeMotor.setPower(power);
            intakeState = 1;
        }
        public void StopIntake() {
            intakeMotor.setPower(0);
            intakeState = 0;
        }

        public void RunToPosIntake(int ticks, double power) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setTargetPosition(-750);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intakeMotor.setPower(0.25);

            while (intakeMotor.isBusy()) {

            }
            intakeMotor.setPower(0);

            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public class Lift {

        public int currentLevel = 1;

        public double liftBottom = 0;
        public double liftAprilTags = 6;
        public double liftLow = 12;
        public double liftMedium = 20;
        public double liftHigh = 28;

        public double liftMotorTicksPerRevolution = 384.5;
        public double liftSpoolDiameter = 7.0/8.0;
        //public double liftCascadeMultiplier = 3;
        public double liftTicksPerInch = liftMotorTicksPerRevolution / (liftSpoolDiameter * Math.PI);

        public double liftPowerUp = 1;
        public double liftPowerDown = 0.7;





        public Lift() {
            liftMotor = hardwareMap.get(DcMotor.class, "lift");
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        public void SetPosition(double liftTargetPosition, double liftPastPosition) {
            liftMotor.setTargetPosition((int) (liftTargetPosition * liftTicksPerInch));
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (liftTargetPosition >= liftPastPosition) {
                liftMotor.setPower(liftPowerUp);
            }
            else {
                liftMotor.setPower(liftPowerDown);

            }

            currentLevel = 0; //edit this later;
        }
    }
    public class SpikeHook {
        public double spikeUp = 0.25;
        public double spikeDropped = 0.7;

        public SpikeHook() {
            spikeHook = hardwareMap.get(Servo.class, "spikeHook");

        }
        public void DropSpike() {
            spikeHook.setPosition(spikeDropped);
        }
        public void ResetSpike() {
            spikeHook.setPosition(spikeUp);
        }
    }
    public class Dropper {

        public boolean dropperOpen = false;
        public double openDropper = 0.12;
        public double closedDropper = 0.31;

        public boolean resetDropper = true;

        public double scoreTimer = 0;


        public Dropper() {

            pixelDropper = hardwareMap.get(Servo.class, "pixelDropper");
        }



        public void CloseDropper() {
            pixelDropper.setPosition(closedDropper);
            dropperOpen = false;
        }
        public void OpenDropper() {
            pixelDropper.setPosition(openDropper);
            dropperOpen = true;
        }

        public void Score() {
            pixelDropper.setPosition(openDropper);
            scoreTimer = gameTimer.milliseconds();

        }
    }

    public class DroneLauncher {
        public double setPosition = 0.69;
        public double releasedPosition = 0.2;
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

    public class PixelSplitter {
        public double openPosition = 0.69;
        public double splitPosition = 0.2;
        public boolean pixelSplitterReset = true;

        public pixelSplitterState splitterState = pixelSplitterState.Open;

        public PixelSplitter() {

            pixelSplitter = hardwareMap.get(Servo.class, "pixelSplitter");

        }

        public void SplitPixel() {
            pixelSplitter.setPosition(splitPosition);
            splitterState = pixelSplitterState.Split;
        }

        public void OpenSplitter() {
            pixelSplitter.setPosition(openPosition);
            splitterState = pixelSplitterState.Open;

        }
    }

    public class Suspension {

        public int suspensionTop = 30;
        public int suspensionBottom = 0;

        public double liftMotorTicksPerRevolution = 28;
        public double liftSpoolDiameter = 0.314961;
        public double liftTicksPerInch = liftMotorTicksPerRevolution / (liftSpoolDiameter * Math.PI);

        public double liftPowerUp = 1;
        public double liftPowerDown = 0.7;

        public double servoUpPosition = 0.5;
        public double servoDownPosition = 0.84;
        public Suspension() {
            suspensionMotor = hardwareMap.get(DcMotor.class, "suspension");
            suspensionMotor.setDirection(DcMotor.Direction.REVERSE);
            suspensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            suspensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            suspensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            suspensionServo = hardwareMap.get(Servo.class, "suspensionServo");
        }

        public void ActivateSuspension() {
            suspensionServo.setPosition(servoUpPosition);
        }
        public void DeactivateSuspension() {
            suspensionServo.setPosition(servoDownPosition);
        }

    }



    public class ContourDetectionPipeline extends OpenCvPipeline {
        public double width;
        public double area;
        public double cX = 0;
        public double cY = 0;


        Mat hierarchy = new Mat();
        Mat hsvFrame = new Mat();
        Mat colorMask = new Mat();


        // Calculate the distance using the formula

        @Override
        public Mat processFrame(Mat input) {
            Mat colorMask = preprocessFrame(input);

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);
                area = calculateArea(largestContour);
                UpdateArea(area);
                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Area: " + String.format("%.2f", area) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();
                //update Center Variables:
                UpdateCenter(cX, cY);
                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {

            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerRed = new Scalar(100, 100, 100);
            Scalar upperRed = new Scalar(180, 255, 255);
            Scalar lowerBlue = new Scalar(0, 100, 100);
            Scalar upperBlue = new Scalar(80, 255, 255);


            if (alliance == "red") {
                Core.inRange(hsvFrame, lowerRed, upperRed, colorMask);
            }
            if (alliance == "blue") {
                Core.inRange(hsvFrame, lowerBlue, upperBlue, colorMask);
            }

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, kernel);

            return colorMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);

            return boundingRect.width;
        }
        public double calculateArea(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);

            return boundingRect.area();
        }



    }

    public class ColorCounter extends OpenCvPipeline {



        Mat hsvFrame = new Mat();
        Mat colorMask = new Mat();
        Mat croppedSectorOne = new Mat();
        Mat croppedSectorTwo = new Mat();
        Mat resizedOne = new Mat();
        Mat resizedTwo = new Mat();



        // Calculate the distance using the formula

        @Override
        public Mat processFrame(Mat input) {
            colorMask = preprocessFrame(input);
            Rect ro1 = new Rect(130,230,250,300);
            Rect ro2 = new Rect(650,220,350,180);
            croppedSectorOne = new Mat(colorMask, ro1);
            croppedSectorTwo = new Mat(colorMask, ro2);

            Size sz1 = new Size(25,20);
            Size sz2 = new Size(35,10);
            Imgproc.resize(croppedSectorOne, resizedOne, sz1);

            Imgproc.resize(croppedSectorTwo, resizedTwo, sz2);
            whiteOne = Core.countNonZero(resizedOne);
            whiteTwo = Core.countNonZero(resizedTwo);

            //return resizedOne;
            Imgproc.rectangle(colorMask, ro1, new Scalar(100, 255, 255), 10);
            Imgproc.rectangle(colorMask, ro2, new Scalar(100, 255, 255), 10);
            return colorMask;
        }

        private Mat preprocessFrame(Mat frame) {

            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerRed = new Scalar(100, 100, 100);
            Scalar upperRed = new Scalar(180, 255, 255);
            Scalar lowerBlue = new Scalar(0, 100, 100);
            Scalar upperBlue = new Scalar(80, 255, 255);


            if (alliance == "red") {
                Core.inRange(hsvFrame, lowerRed, upperRed, colorMask);
            }
            if (alliance == "blue") {
                Core.inRange(hsvFrame, lowerBlue, upperBlue, colorMask);
            }

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, kernel);

            return colorMask;
        }





    }

    private  double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    private void UpdateCenter(double cx, double cy) {
        centerX = cx;
        centerY = cy;
    }

    private void UpdateArea(double areaImport) {
        area = areaImport;
    }
    public int ScanForElement(int preferredFailOutput) {
        int returnSpike = preferredFailOutput;
        if (centerX <= (STREAM_WIDTH / 2.0)) {
            returnSpike = 1;
        }
        if (centerX > (STREAM_WIDTH / 2.0)) {
            returnSpike = 2;
        }
        if (area < 35250) {
            returnSpike = 3;
        }
        return returnSpike;
    }

    public int ScanForElementBitmap(int preferredFailOutput) {
        int returnSpike = preferredFailOutput;

        if (whiteOne > whiteTwo && whiteOne > 80) {
            returnSpike = 1;
        } else if (whiteTwo > whiteOne && whiteTwo > 80) {
            returnSpike = 2;
        } else {
            returnSpike = 3;
        }

        return returnSpike;
    }
    public double getCenter() {
        return centerX;
    }


}