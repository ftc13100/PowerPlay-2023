package com.example.pathplanning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class PathPlanning {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(35.25, -62, Math.toRadians(90));
        Pose2d locOne = new Pose2d(11.75, -23.5, Math.toRadians(-90));
        Pose2d locTwo = new Pose2d(35.25, -23.5, Math.toRadians(-90));
        Pose2d locThree = new Pose2d(58.75, -23.5 - 11.75, Math.toRadians(180));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15).build();

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .addEntity(bot)
                .start();
    }
}