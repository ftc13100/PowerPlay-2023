package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

class DriveSubsystem(
    private val drive: SampleMecanumDrive,
    private val fieldCentric: Boolean
) : SubsystemBase() {
    fun setMode(mode: DcMotor.RunMode?) = drive.setMode(mode)

    fun setPIDFCoefficients(mode: DcMotor.RunMode?, coefficients: PIDFCoefficients?) =
        drive.setPIDFCoefficients(mode, coefficients)

    fun update() = drive.update()

    fun updatePoseEstimate() = drive.updatePoseEstimate()

    fun drive(leftY: Double, leftX: Double, rightX: Double) {
        val (_, _, heading) = poseEstimate!!
        val (x, y) = Vector2d(-leftY, -leftX).rotated(
            if (fieldCentric) -heading else 0.0
        )
        drive.setWeightedDrivePower(
            Pose2d(
                x,
                y,
                -rightX
            )
        )
    }

    fun turn(radians: Double) = drive.turnAsync(radians)

    fun setDrivePower(drivePower: Pose2d?) = drive.setDrivePower(drivePower!!)

    var poseEstimate: Pose2d
        get() = drive.poseEstimate
        set(pose) {
            drive.poseEstimate = pose
        }

    fun trajectoryBuilder(startPose: Pose2d?): TrajectoryBuilder =
        drive.trajectoryBuilder(startPose)

    fun trajectoryBuilder(startPose: Pose2d?, reversed: Boolean): TrajectoryBuilder =
        drive.trajectoryBuilder(startPose, reversed)

    fun trajectoryBuilder(startPose: Pose2d?, startHeading: Double): TrajectoryBuilder =
        drive.trajectoryBuilder(startPose, startHeading)

    fun followTrajectory(trajectory: Trajectory?) = drive.followTrajectoryAsync(trajectory)

    fun stop() = drive(0.0, 0.0, 0.0)

    val isBusy: Boolean
        get() = drive.isBusy

    val wheelVelocities: List<Double>?
        get() = drive.getWheelVelocities()

    val poseVelocity: Pose2d?
        get() = drive.poseVelocity

    val localizer: Localizer
        get() = drive.localizer
}