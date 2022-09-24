package com.example.pathplanning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PathPlanning {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(35.25, -62, Math.toRadians(90));
        Pose2d prelimOne = new Pose2d(11.75, -11.75);
        Pose2d locOne = new Pose2d(11.75, -23.5);
        Pose2d prelimTwo = new Pose2d(35.25, -11.75);
        Pose2d locTwo = new Pose2d(35.25, -23.5);
        Pose2d prelimThree = new Pose2d(58.75, -11.75);
        Pose2d locThree = new Pose2d(58.75, -23.5, Math.toRadians(-90));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(startPose)
                            .splineToConstantHeading(new Vector2d(35.25, -15), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(23.5, -9.5), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(35.25, -15), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(35.25, -35.25), Math.toRadians(90))
                            .splineToSplineHeading(locThree, Math.toRadians(90))
                            .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .addEntity(bot)
                .start();
    }
}