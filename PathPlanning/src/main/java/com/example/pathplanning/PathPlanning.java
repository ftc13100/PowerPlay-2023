package com.example.pathplanning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PathPlanning {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-35.25, -62.0, Math.toRadians(90.0));
        Pose2d loc3 = new Pose2d(-11.75, -23.5, Math.toRadians(-90.0));
        Pose2d loc2 = new Pose2d(-35.25, -23.5, Math.toRadians(-90.0));
        Pose2d loc1 = new Pose2d(-58.75, -35.25, Math.toRadians(0));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineToConstantHeading(new Vector2d(-35.25, -15.0), Math.toRadians(90.0))
                                .splineToConstantHeading(new Vector2d(-23.5, -9.5), Math.toRadians(90.0))
                                .setReversed(true)
                                // loc 1
//                                .splineToConstantHeading(new Vector2d(-35.25, -15.0), Math.toRadians(-90.0))
//                                .splineToConstantHeading(new Vector2d(-35.25, -26.0), Math.toRadians(-90.0))
//                                .splineToSplineHeading(loc1, Math.toRadians(180.0))
                                // loc 2
//                                .splineToConstantHeading(new Vector2d(-35.25, -15.0), Math.toRadians(-90.0))
//                                .splineToSplineHeading(loc2, Math.toRadians(-90.0))
                                // loc 3
//                                .splineToConstantHeading(new Vector2d(-11.75, -15.0), Math.toRadians(-90.0))
//                                .splineToSplineHeading(loc3, Math.toRadians(-90.0))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .addEntity(bot)
                .start();
    }
}