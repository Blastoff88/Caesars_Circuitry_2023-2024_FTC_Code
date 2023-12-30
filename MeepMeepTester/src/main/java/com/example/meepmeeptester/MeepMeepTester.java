package com.example.meepmeeptester;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepTester {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, 2.216, Math.toRadians(60), 17.4)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(10, 60, Math.toRadians(90)))//auto 1
                                        .back(20)
                                        .splineTo(new Vector2d(12,30), Math.toRadians(0))
//                                        .splineTo(new Vector2d(10,12), Math.toRadians(270))
                                        .build()
//                        drive.trajectorySequenceBuilder(new Pose2d(10, 60, Math.toRadians(270)))//auto 1
//                                .forward(30)
//                                .back(5)
//                                .splineToSplineHeading(new Pose2d(50, 35, Math.toRadians(180)), Math.toRadians(0))
//                                .build()

//                                drive.trajectorySequenceBuilder(new Pose2d(10, 60, Math.toRadians(270)))//auto 2
//                                .splineTo(new Vector2d(10,30), Math.toRadians(270))
//                                .back(10)
//                                .splineToSplineHeading(new Pose2d(56, 10, Math.toRadians(180)), Math.toRadians(0))
//                                .build()

//                                  drive.trajectorySequenceBuilder(new Pose2d(-34, 60, Math.toRadians(270)))//auto 3
//                                          .splineTo(new Vector2d(-34,33), Math.toRadians(270))
//                                          .back(3)
//                                          .strafeLeft(30)
//                                          .splineToSplineHeading(new Pose2d(56, 60, Math.toRadians(180)), Math.toRadians(0))
//                                          .build()

//                                drive.trajectorySequenceBuilder(new Pose2d(-34, 60, Math.toRadians(270)))//auto 4
//                                        .splineTo(new Vector2d(-34,33), Math.toRadians(270))
//                                        .back(3)
//                                        .strafeLeft(50)
//                                        .splineToSplineHeading(new Pose2d(56, 10, Math.toRadians(180)), Math.toRadians(0))
//                                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}