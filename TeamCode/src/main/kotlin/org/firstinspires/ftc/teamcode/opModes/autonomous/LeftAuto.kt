package org.firstinspires.ftc.teamcode.opModes.autonomous

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.constants.AprilTagCamera
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.AprilTagDetectionPipeline
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous(name = "Left Auto")
class LeftAuto : OpMode() {
    // Constants
    private val startPose = Pose2d(-35.25, -62.0, Math.toRadians(90.0))
    private val loc1 = Pose2d(-58.75, -35.25, Math.toRadians(0.0))
    private val loc2 = Pose2d(-35.25, -23.5, Math.toRadians(-90.0))
    private val loc3 = Pose2d(-11.75, -23.5, Math.toRadians(-90.0))

    // Hardware
    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor
    private lateinit var intake: Motor
    private lateinit var limit: TouchSensor

    // Subsystems
    private lateinit var slidesSubsystem: SlidesSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    private lateinit var drive: SampleMecanumDrive

    // Paths
    private lateinit var zoneOnePath: TrajectorySequence
    private lateinit var zoneTwoPath: TrajectorySequence
    private lateinit var zoneThreePath: TrajectorySequence

    private lateinit var detectedTags: List<AprilTagDetection>
    private lateinit var pipeline: AprilTagDetectionPipeline

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
        pipeline = AprilTagDetectionPipeline(
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
        slidesLeft = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName)
        slidesRight = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName)
        intake = Motor(hardwareMap, DeviceConfig.INTAKE.deviceName)
        limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)

        slidesSubsystem = SlidesSubsystem(slidesLeft, slidesRight, limit, telemetry)
        intakeSubsystem = IntakeSubsystem(intake)
        drive.poseEstimate = startPose

        // Paths
        zoneOnePath = drive.trajectorySequenceBuilder(startPose)
            .addTemporalMarker(0.0) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.INTAKE) }
            .waitSeconds(0.5)
            .splineToConstantHeading(Vector2d(-35.25, -16.0), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-21.0, -11.5), Math.toRadians(90.0))
            .waitSeconds(1.5)
            .addTemporalMarker(3.5) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.HIGH) }
            .splineToConstantHeading(Vector2d(-21.0, -7.5), Math.toRadians(90.0))
            .addTemporalMarker(6.0) { intakeSubsystem.outtake() }
            .addTemporalMarker(6.5) { intakeSubsystem.stop() }
            .waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-21.0, -11.5), Math.toRadians(90.0))
            .addTemporalMarker(7.5) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.GROUND) }
            .waitSeconds(3.0)
            .setReversed(true)
            .splineToConstantHeading(Vector2d(-35.25, -15.0), Math.toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-35.25, -26.0), Math.toRadians(-90.0))
            .splineToSplineHeading(loc1, Math.toRadians(180.0))
            .build()

        zoneTwoPath = drive.trajectorySequenceBuilder(startPose)
            .addTemporalMarker(0.0) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.INTAKE) }
            .waitSeconds(0.5)
            .splineToConstantHeading(Vector2d(-35.25, -16.0), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-21.0, -11.5), Math.toRadians(90.0))
            .waitSeconds(1.5)
            .addTemporalMarker(3.5) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.HIGH) }
            .splineToConstantHeading(Vector2d(-21.0, -7.5), Math.toRadians(90.0))
            .addTemporalMarker(6.0) { intakeSubsystem.outtake() }
            .addTemporalMarker(6.5) { intakeSubsystem.stop() }
            .waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-21.0, -11.5), Math.toRadians(90.0))
            .addTemporalMarker(7.5) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.GROUND) }
            .waitSeconds(3.0)
            .setReversed(true)
            .splineToConstantHeading(Vector2d(-35.25, -16.0), Math.toRadians(-90.0))
            .splineToSplineHeading(loc2, Math.toRadians(-90.0))
            .build()

        zoneThreePath = drive.trajectorySequenceBuilder(startPose)
            .addTemporalMarker(0.0) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.INTAKE) }
            .waitSeconds(0.5)
            .splineToConstantHeading(Vector2d(-35.25, -16.0), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-21.0, -11.5), Math.toRadians(90.0))
            .waitSeconds(1.5)
            .addTemporalMarker(3.5) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.HIGH) }
            .splineToConstantHeading(Vector2d(-21.0, -7.5), Math.toRadians(90.0))
            .addTemporalMarker(6.0) { intakeSubsystem.outtake() }
            .addTemporalMarker(6.5) { intakeSubsystem.stop() }
            .waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-21.0, -11.5), Math.toRadians(90.0))
            .addTemporalMarker(7.5) { slidesSubsystem.setTargetPosition(SlidesConst.SlidesPosition.GROUND) }
            .waitSeconds(3.0)
            .setReversed(true)
            .splineToConstantHeading(Vector2d(-11.75, -15.0), Math.toRadians(-90.0))
            .splineToSplineHeading(loc3, Math.toRadians(-90.0))
            .build()

        // Vision detection
        detectedTags = pipeline.getLatestResults()
    }

    override fun init_loop() {
        detectedTags = pipeline.getLatestResults()
    }

    override fun start() {
        // Vision-based path assignment
        val path = if (detectedTags.isNotEmpty()) {
            when (detectedTags[0].id) {
                1213 -> zoneTwoPath
                302 -> zoneThreePath
                // 1021 for Zone One
                else -> zoneOnePath
            }
        } else {
            zoneOnePath
        }

        drive.followTrajectorySequenceAsync(path)
    }

    override fun loop() {
        drive.update()

        if (slidesSubsystem.atTargetPosition()) {
            if (slidesSubsystem.getTargetPosition() == SlidesConst.SlidesPosition.GROUND) {
                slidesSubsystem.stop()
            } else {
                slidesSubsystem.stall()
            }
        } else {
            slidesSubsystem.operateSlides()
        }
    }
}