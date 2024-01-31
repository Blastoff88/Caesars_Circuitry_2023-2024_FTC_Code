package org.firstinspires.ftc.teamcode.Autonomous.Cup2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
@Disabled
public class RoadrunnerAuto1withDis extends LinearOpMode {
    private Servo Yellow;
    private Servo wall;
    private DistanceSensor sensorDistanceM;
    private DistanceSensor sensorDistanceL;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;
    public static int Auto = 0;
    @Override
    public void runOpMode(){
        Yellow = hardwareMap.get(Servo.class, "Yellow");
        wall = hardwareMap.get(Servo.class, "Wall");
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        wall.setPosition(1);
        sensorDistanceM = hardwareMap.get(DistanceSensor.class, "centerDis");
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "leftDis");
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistanceM;
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) sensorDistanceL;


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
                .strafeRight(20)
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
                .strafeRight(27)
                .build();
        TrajectorySequence TrajSeqR = drivetrain.trajectorySequenceBuilder(new Pose2d(10,64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10,20,Math.toRadians(180)))
                .lineTo(new Vector2d(-5,20))
                .addDisplacementMarker(()->{
                    wall.setPosition(0);
                })
                .back(5)
                .addDisplacementMarker(()->{
                    wall.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(10,20,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(60,17, Math.toRadians(180)))

                .addTemporalMarker(6,()->{
                    Yellow.setPosition(.45);
                })
                .addTemporalMarker(6.5,()->{
                    Yellow.setPosition(0);
                })
                .forward(5)
                .strafeRight(35)
                .build();

        waitForStart();
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        if(!isStopRequested()){
            telemetry.addData("range", String.format("%.01f in", sensorDistanceM.getDistance(DistanceUnit.INCH)));
            telemetry.update();
//            sleep(1000);
            if(sensorDistanceM.getDistance(DistanceUnit.INCH)>=12 && sensorDistanceM.getDistance(DistanceUnit.INCH)<=14) {
                greenLED.setState(true);
                redLED.setState(false);
                drivetrain.followTrajectorySequence(TrajSeqM);
            } else if (sensorDistanceL.getDistance(DistanceUnit.INCH)>=15 && sensorDistanceL.getDistance(DistanceUnit.INCH)<=20) {
                redLED.setState(false);
                greenLED.setState(true);
                telemetry.addData("range", String.format("%.01f in", sensorDistanceL.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                drivetrain.followTrajectorySequence(TrajSeqL);
            } else {
                redLED.setState(true);
                greenLED.setState(false);
                drivetrain.followTrajectorySequence(TrajSeqR);
            }

        }
    }
}
