package com.example.pathplanning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PathPlanning {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(-35.25, -62.0, Math.toRadians(90.0));

        DefaultBotBuilder base = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15)
                .setConstraints(38.110287416570166, 38.110287416570166, Math.toRadians(457.2273162437774), Math.toRadians(138.19991297468354), 15.2);

        RoadRunnerBotEntity bot = originalRedAuto(base, startPose);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .addEntity(bot)
                .start();
    }

    public static RoadRunnerBotEntity originalRedAuto(DefaultBotBuilder bot, Pose2d startPose) {
        Pose2d loc3 = new Pose2d(-11.75, -11.75, Math.toRadians(180.0));
        Pose2d loc2 = new Pose2d(-35.25, -11.75, Math.toRadians(180.0));
        Pose2d loc1 = new Pose2d(-58.75, -11.75, Math.toRadians(180.0));

        return bot.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineToConstantHeading(new Vector2d(-35.25, -11.75), Math.toRadians(90.0))
                                .splineToConstantHeading(new Vector2d(-32.25, -8.75), Math.toRadians(90.0))
                                .splineToLinearHeading(new Pose2d(-35.25, -11.75, Math.toRadians(180.0)), Math.toRadians(180.0))
                                .splineToConstantHeading(loc3.vec(), Math.toRadians(180.0))
                                // loc 1
//                                .splineToConstantHeading(new Vector2d(-35.25, -15.0), Math.toRadians(-90.0))
//                                .splineToConstantHeading(new Vector2d(-35.25, -26.0), Math.toRadians(-90.0))
//                                .splineToSplineHeading(loc1, Math.toRadians(180.0))
                                // loc 2
//                                .splineToConstantHeading(new Vector2d(-35.25, -15.0), Math.toRadians(-90.0))
//                                .splineToSplineHeading(loc2, Math.toRadians(-90.0))
//                                 loc 3
//                                .splineToConstantHeading(new Vector2d(-11.75, -15.0), Math.toRadians(-90.0))
//                                .splineToLinearHeading(loc3, Math.toRadians(-90.0))
                                .build()
        );
    }

    public static RoadRunnerBotEntity redAutoTwoCycle(DefaultBotBuilder bot, Pose2d startPose) {
        Pose2d stackPose = new Pose2d(-65, -12, Math.toRadians(180));
        Pose2d loc3 = new Pose2d(-11.75, -23.5, Math.toRadians(-90.0));
        Pose2d loc2 = new Pose2d(-35.25, -23.5, Math.toRadians(-90.0));
        Pose2d loc1 = new Pose2d(-58.75, -35.25, Math.toRadians(0));

        return bot.followTrajectorySequence(
                drive -> drive.trajectorySequenceBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(-35.25, -15.0), Math.toRadians(90.0))
                        .splineToConstantHeading(new Vector2d(-23.5, -9.5), Math.toRadians(90.0))
                        .waitSeconds(2)
                        .splineToSplineHeading(stackPose, Math.toRadians(180))
                        .waitSeconds(2)
                        .splineToSplineHeading(new Pose2d(-23.5, -9.5, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(2)
                        .splineToSplineHeading(stackPose, Math.toRadians(180))
                        .waitSeconds(2)
                        .splineToSplineHeading(new Pose2d(-23.5, -9.5, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(2)

//                        .lineToLinearHeading(stackPose)
//                        .waitSeconds(2)
//                        .lineToSplineHeading(new Pose2d(-23.5, -9.5, Math.toRadians(90)))
//                        .waitSeconds(2)
                        .splineToConstantHeading(new Vector2d(-35.25, -15.0), Math.toRadians(-90.0))
                        .splineToConstantHeading(new Vector2d(-35.25, -26.0), Math.toRadians(-90.0))
                        .splineToSplineHeading(loc1, Math.toRadians(180.0))
                        .build()
        );
    }
}