package org.firstinspires.ftc.teamcode.Autonomous.Cup2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
@Disabled
public class RoadrunnerAuto2 extends LinearOpMode {
    private Servo Yellow;
    private Servo wall;
    public static int Auto = 1;
    @Override
    public void runOpMode(){
        Yellow = hardwareMap.get(Servo.class, "Yellow");
        wall = hardwareMap.get(Servo.class, "Wall");
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        wall.setPosition(1);


        drivetrain.setPoseEstimate(new Pose2d(-20,60,Math.toRadians(90)));
        TrajectorySequence TrajSeqL = drivetrain.trajectorySequenceBuilder(new Pose2d(-20,64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-20,20,Math.toRadians(0)))
                .lineTo(new Vector2d(-32,20))
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
        TrajectorySequence TrajSeqM = drivetrain.trajectorySequenceBuilder(new Pose2d(-20,64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-20,12,Math.toRadians(270)))
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
        TrajectorySequence TrajSeqR = drivetrain.trajectorySequenceBuilder(new Pose2d(-20,64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-20,20,Math.toRadians(180)))
                .lineTo(new Vector2d(-15,20))
                .addDisplacementMarker(()->{
                wall.setPosition(0);
                })
                .back(5)
                .addDisplacementMarker(()->{
                    wall.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(-20,20,Math.toRadians(180)))
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

        if(!isStopRequested()){
            if(Auto == 1){
                drivetrain.followTrajectorySequence(TrajSeqL);
            } else if (Auto == 2) {
                drivetrain.followTrajectorySequence(TrajSeqM);
            } else if (Auto == 3) {
                drivetrain.followTrajectorySequence(TrajSeqR);
            }

        }
    }
}
