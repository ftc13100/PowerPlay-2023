package org.firstinspires.ftc.teamcode.opModes.autonomous

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.constants.AprilTagCamera
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.AprilTagDetectionPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous(name = "Red Right Auto")
class RedRightAuto : OpMode() {
    private val startPose = Pose2d(35.25, -62.0, Math.toRadians(90.0))
    private val loc1 = Pose2d(11.75, -23.5, Math.toRadians(-90.0))
    private val loc2 = Pose2d(35.25, -23.5, Math.toRadians(-90.0))
    private val loc3 = Pose2d(58.75, -35.25, Math.toRadians(180.0))

    private var drive: SampleMecanumDrive? = null

    @SuppressLint("DiscouragedApi")
    override fun init() {
        // Vision Init
        val monitorId: Int = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val webcamName: WebcamName = hardwareMap.get(WebcamName::class.java, DeviceConfig.VISION_CAMERA.deviceName)
        val camera: OpenCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, monitorId)
        val pipeline = AprilTagDetectionPipeline(
            AprilTagCamera.TAGSIZE.value,
            AprilTagCamera.FX.value,
            AprilTagCamera.FY.value,
            AprilTagCamera.CX.value,
            AprilTagCamera.CY.value
        )

        camera.setPipeline(pipeline)

        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
            }
        })

        // Hardware Init
        drive = SampleMecanumDrive(hardwareMap)

        // Paths
        val pathBuilder: TrajectorySequenceBuilder = drive!!.trajectorySequenceBuilder(startPose)
            .splineToConstantHeading(Vector2d(35.25, -15.0), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(23.5, -9.5), Math.toRadians(90.0))
            .setReversed(true)

        val zoneOnePath: Trajectory = drive!!.trajectoryBuilder(Pose2d(23.5, -9.5), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(11.75, -15.0), Math.toRadians(-90.0))
            .splineToSplineHeading(loc1, Math.toRadians(-90.0))
            .build()

        val zoneTwoPath: Trajectory = drive!!.trajectoryBuilder(Pose2d(23.5, -9.5), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(35.25, -15.0), Math.toRadians(-90.0))
            .splineToSplineHeading(loc2, Math.toRadians(-90.0))
            .build()

        val zoneThreePath: Trajectory = drive!!.trajectoryBuilder(Pose2d(23.5, -9.5), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(35.25, -15.0), Math.toRadians(-90.0))
            .splineToConstantHeading(Vector2d(35.25, -26.0), Math.toRadians(-90.0))
            .splineToSplineHeading(loc3, Math.toRadians(0.0))
            .build()

        // Vision-based path assignment
        val signalPath: Trajectory = when (pipeline.getLatestResults()[0].id) {
            1213 -> zoneTwoPath
            301 -> zoneThreePath
            // 1021 for Zone One
            else -> zoneOnePath
        }

        val path: TrajectorySequence = pathBuilder.addTrajectory(signalPath).build()

        drive!!.followTrajectorySequenceAsync(path)
    }

    override fun loop() {
        drive!!.update()
    }
}