package com.example.pathplanning

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

object PathPlanning {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(600)
        val startPose = Pose2d(-35.25, -62.0, Math.toRadians(90.0))
        val base = DefaultBotBuilder(meepMeep)
            .setDimensions(15.0, 15.0)
            .setConstraints(
                38.110287416570166,
                38.110287416570166,
                Math.toRadians(457.2273162437774),
                Math.toRadians(138.19991297468354),
                13.1 // in

            )

        val parkBot1 = base.parkPreloadAuto(startPose, ParkLocations.LOCATION_1)
        val parkBot2 = base.parkPreloadAuto(startPose, ParkLocations.LOCATION_2)
        val parkBot3 = base.parkPreloadStackAuto(startPose, ParkLocations.LOCATION_2, 3)

        meepMeep.setBackground(Background.FIELD_POWERPLAY_OFFICIAL)
//            .addEntity(parkBot1)
//            .addEntity(parkBot2)
            .addEntity(parkBot3)
            .start()
    }


    private fun DefaultBotBuilder.parkAuto(
        startPose: Pose2d,
        loc: ParkLocations,
    ): RoadRunnerBotEntity = followTrajectorySequence {
            it.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(loc.pose.vec(), Math.toRadians(90.0))
                .splineToConstantHeading(Vector2d(loc.pose.x, -23.5), Math.toRadians(90.0))
                // OLD
                //                                .splineToConstantHeading(new Vector2d(-35.25, -11.75), Math.toRadians(90.0))
                //                                .splineToConstantHeading(new Vector2d(-32.25, -8.75), Math.toRadians(90.0))
                //                                .splineToLinearHeading(new Pose2d(-35.25, -11.75, Math.toRadians(180.0)), Math.toRadians(180.0))
                //                                .splineToConstantHeading(loc3.vec(), Math.toRadians(180.0))
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
        }
    private fun DefaultBotBuilder.parkPreloadAuto(
        startPose: Pose2d,
        loc: ParkLocations,
    ): RoadRunnerBotEntity {
        val (stackX, stackY, stackHeading) = Pose2d(-60.0, -12.0, Math.toRadians(180.0))
        val (startX, startY, startHeading) = startPose

        return followTrajectorySequence {
            it.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(Vector2d(-34.0, 0.0), 90.0.toRadians())
//                .splineToConstantHeading(Vector2d(stackX, startY), Math.toRadians(90.0))
                .waitSeconds(2.0)
                .lineToSplineHeading(Pose2d(startX, stackY, stackHeading))
                .apply {
                    if (loc != ParkLocations.LOCATION_2) {
                        lineToConstantHeading(Vector2d(loc.pose.x, stackY))
                    }
                }
                .build()
        }
    }

    private fun DefaultBotBuilder.parkPreloadStackAuto(
        startPose: Pose2d,
        loc: ParkLocations,
        cycle: Int,
    ) : RoadRunnerBotEntity {
        assert(cycle > 0)
        val (stackX, stackY, stackHeading) = Pose2d(-60.0, -12.0, Math.toRadians(180.0))
        val (startX, startY, startHeading) = startPose

        return followTrajectorySequence {
            it.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(Vector2d(-34.0, 0.0), 90.0.toRadians())
                .apply {
                    for(i in 1..cycle) {
                        waitSeconds(0.7)
                            .lineToSplineHeading(Pose2d(startX, stackY, stackHeading))
                            .lineToConstantHeading(Vector2d(stackX, stackY))
                            .waitSeconds(0.7)
                            .lineToSplineHeading(Pose2d(startX, stackY, startHeading))
                            .splineToConstantHeading(Vector2d(-34.0, 0.0), 90.0.toRadians())
                    }
                }
                .waitSeconds(1.0)
                .lineToSplineHeading(Pose2d(startX, stackY, stackHeading))
                .apply {
                    if (loc != ParkLocations.LOCATION_2) {
                        lineToConstantHeading(Vector2d(loc.pose.x, stackY))
                    }
                }
                .build()
        }
    }

    fun redAutoTwoCycle(bot: DefaultBotBuilder, startPose: Pose2d): RoadRunnerBotEntity {
        val stackPose = Pose2d(-65.0, -12.0, Math.toRadians(180.0))
        val (x, y, heading) = Pose2d(-11.75, -23.5, Math.toRadians(-90.0))
        val (x1, y1, heading1) = Pose2d(-35.25, -23.5, Math.toRadians(-90.0))
        val loc1 = Pose2d(-58.75, -35.25, Math.toRadians(0.0))
        return bot.followTrajectorySequence { drive: DriveShim ->
            drive.trajectorySequenceBuilder(
                startPose
            )
                .splineToConstantHeading(Vector2d(-35.25, -15.0), Math.toRadians(90.0))
                .splineToConstantHeading(Vector2d(-23.5, -9.5), Math.toRadians(90.0))
                .waitSeconds(2.0)
                .splineToSplineHeading(stackPose, Math.toRadians(180.0))
                .waitSeconds(2.0)
                .splineToSplineHeading(
                    Pose2d(-23.5, -9.5, Math.toRadians(90.0)),
                    Math.toRadians(90.0)
                )
                .waitSeconds(2.0)
                .splineToSplineHeading(stackPose, Math.toRadians(180.0))
                .waitSeconds(2.0)
                .splineToSplineHeading(
                    Pose2d(-23.5, -9.5, Math.toRadians(90.0)),
                    Math.toRadians(90.0)
                )
                .waitSeconds(2.0) //                        .lineToLinearHeading(stackPose)
                //                        .waitSeconds(2)
                //                        .lineToSplineHeading(new Pose2d(-23.5, -9.5, Math.toRadians(90)))
                //                        .waitSeconds(2)
                .splineToConstantHeading(Vector2d(-35.25, -15.0), Math.toRadians(-90.0))
                .splineToConstantHeading(Vector2d(-35.25, -26.0), Math.toRadians(-90.0))
                .splineToSplineHeading(loc1, Math.toRadians(180.0))
                .build()
        }
    }

    enum class ParkLocations(val pose: Pose2d) {
        LOCATION_1(Pose2d(-54.75, -62.0, Math.toRadians(90.0))),
        LOCATION_2(Pose2d(-35.25, -60.0, Math.toRadians(90.0))),
        LOCATION_3(Pose2d(-14.75, -62.0, Math.toRadians(90.0)))
    }
}