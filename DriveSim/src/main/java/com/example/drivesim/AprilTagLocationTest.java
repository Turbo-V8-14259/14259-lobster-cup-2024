package com.example.drivesim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AprilTagLocationTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: 60, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->//drive to each apriltag on backdrop
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .lineToLinearHeading(new Pose2d(60.25F, 41.41F))
                                .lineToLinearHeading(new Pose2d(60.25F, 35.41F))
                                .lineToLinearHeading(new Pose2d(60.25F, 29.41F))
                                .lineToLinearHeading(new Pose2d(60.25F, -29.41F))
                                .lineToLinearHeading(new Pose2d(60.25F, -35.41F))
                                .lineToLinearHeading(new Pose2d(60.25F, -41.41F))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}