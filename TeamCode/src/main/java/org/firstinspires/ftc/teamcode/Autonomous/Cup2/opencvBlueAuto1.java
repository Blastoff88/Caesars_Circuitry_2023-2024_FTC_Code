package org.firstinspires.ftc.teamcode.Autonomous.Cup2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "blueLeftOuter")
@Disabled
public class opencvBlueAuto1 extends LinearOpMode {
    private Servo Yellow;
    private Servo wall;
    public static int Auto = 0;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.125;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


    @Override
    public void runOpMode() {
        Yellow = hardwareMap.get(Servo.class, "Yellow");
        wall = hardwareMap.get(Servo.class, "Wall");
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        wall.setPosition(1);
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        drivetrain.setPoseEstimate(new Pose2d(10,60,Math.toRadians(90)));
        TrajectorySequence TrajSeqL = drivetrain.trajectorySequenceBuilder(new Pose2d(10,64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10,20,Math.toRadians(0)))
                .lineTo(new Vector2d(22,20))
                .addDisplacementMarker(()->{
                    wall.setPosition(0);
                })
                .back(10)
                .addDisplacementMarker(()->{
                    wall.setPosition(1);
                })
                .strafeLeft(30)
                .lineToLinearHeading(new Pose2d(66,36, Math.toRadians(180)))
                .addTemporalMarker(8.4,()->{
                    Yellow.setPosition(.45);
                })
                .addTemporalMarker(9.1,()->{
                    Yellow.setPosition(0);
                })
                .forward(5)
                .strafeRight(27)
                .build();
        TrajectorySequence TrajSeqM = drivetrain.trajectorySequenceBuilder(new Pose2d(10,64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10,12,Math.toRadians(270)))
                .addDisplacementMarker(()->{
                    wall.setPosition(0);
                })
                .back(10)
                .addDisplacementMarker(()->{
                    wall.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(64, 24, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(6,()->{
                    Yellow.setPosition(.45);
                })
                .addTemporalMarker(6.9,()->{
                    Yellow.setPosition(0);
                })
                .forward(3)
                .strafeRight(34)
                .build();
        TrajectorySequence TrajSeqR = drivetrain.trajectorySequenceBuilder(new Pose2d(10,64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10,20,Math.toRadians(180)))
                .lineTo(new Vector2d(-3,20))
                .addDisplacementMarker(()->{
                    wall.setPosition(0);
                })
                .back(5)
                .addDisplacementMarker(()->{
                    wall.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(10,20,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(58,17, Math.toRadians(180)))

                .addTemporalMarker(6,()->{
                    Yellow.setPosition(.45);
                })
                .addTemporalMarker(6.5,()->{
                    Yellow.setPosition(0);
                })
                .forward(5)
                .strafeRight(43)
                .build();

        waitForStart();
        controlHubCam.stopStreaming();
//        while (opModeIsActive()) {
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.update();
//
//            // The OpenCV pipeline automatically processes frames and handles detection
//        }
        if(!isStopRequested()) {
//            sleep(1000);
            if (cX>10 && cX<=100) {
                drivetrain.followTrajectorySequence(TrajSeqL);
            } else if (cX>100 && cX<500) {
                drivetrain.followTrajectorySequence(TrajSeqM);
            } else {
                drivetrain.followTrajectorySequence(TrajSeqR);
            }
        }
        controlHubCam.closeCameraDevice();

        // Release resources


    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        telemetry.addData("OpenCV intialized", true);
        telemetry.update();
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        Mat hierarchy = new Mat();
        Mat hsvFrame = new Mat();
        Mat yellowMask = new Mat();
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {

            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow = new Scalar(100,150,0);
            Scalar upperYellow = new Scalar(140,255,255);

            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
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

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }


}
