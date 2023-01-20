package org.firstinspires.ftc.teamcode.opModes.autonomous.left

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.constants.AprilTagCamera
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.AprilTagDetectionPipeline
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous(name = "Left Auto", preselectTeleOp = "Main" )
class LeftParkAuto : OpMode() {

    // Constants
    private val startPose = Pose2d(-35.25, -62.0, Math.toRadians(90.0))
    private val loc3 = Pose2d(-14.75, -62.0, Math.toRadians(90.0))
    private val loc2 = Pose2d(-35.25, -60.0, Math.toRadians(90.0))
    private val loc1 = Pose2d(-54.75, -62.0, Math.toRadians(90.0))

    // Hardware
    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor
    private lateinit var limit: TouchSensor
    private lateinit var rotationServo: Servo
    private lateinit var clawServo: Servo

    // Subsystems
    private lateinit var slidesClawSubsystem: SlidesClawSubsystem
    private lateinit var drive: SampleMecanumDrive

    // Paths
    private lateinit var zoneOnePath: TrajectorySequence
    private lateinit var zoneTwoPath: TrajectorySequence
    private lateinit var zoneThreePath: TrajectorySequence

    private lateinit var detectedTags: List<AprilTagDetection>
    private lateinit var pipeline: AprilTagDetectionPipeline
    private lateinit var path: TrajectorySequence

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
        telemetry.addData("Status", "Initializing Hardware")
        telemetry.update()
        drive = SampleMecanumDrive(hardwareMap)
        slidesLeft = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName)
        slidesRight = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName)
        limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)
        clawServo = hardwareMap.get(Servo::class.java, DeviceConfig.CLAW_SERVO.deviceName)
        rotationServo = hardwareMap.get(Servo::class.java, DeviceConfig.ROTATION_SERVO.deviceName)

        slidesClawSubsystem = SlidesClawSubsystem(slidesLeft, slidesRight, clawServo, rotationServo, limit, telemetry)
        slidesClawSubsystem.closeClaw()
        drive.poseEstimate = startPose

        // Paths
        telemetry.addData("Status", "Initializing Paths 1")
        telemetry.update()

        zoneOnePath = drive.trajectorySequenceBuilder(startPose)
            .splineToConstantHeading(loc1.vec(), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-54.75, -23.5), Math.toRadians(90.0))
            .build()

        zoneTwoPath = drive.trajectorySequenceBuilder(startPose)
            .splineToConstantHeading(loc2.vec(), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-35.25, -23.5), Math.toRadians(90.0))
            .build()

        zoneThreePath = drive.trajectorySequenceBuilder(startPose)
            .splineToConstantHeading(loc3.vec(), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-14.75, -23.5), Math.toRadians(90.0))
            .build()

        // Vision detection
        detectedTags = pipeline.getLatestResults()
        path = zoneOnePath
    }

    override fun init_loop() {
        detectedTags = pipeline.getLatestResults()

        if (detectedTags.isNotEmpty()) {
            path = when (detectedTags[0].id) {
                1213 -> zoneTwoPath
                302 -> zoneThreePath
                // 1021 for Zone One
                else -> zoneOnePath
            }
        }

        telemetry.addData("Status", "Initializing Paths ${
            when(path) {
                zoneOnePath -> "1"
                zoneTwoPath -> "2"
                zoneThreePath -> "3"
                else -> "Default: 1"
            }
        }")
        telemetry.update()
    }

    override fun start() {
        drive.followTrajectorySequenceAsync(path)
    }

    override fun loop() {
        drive.update()

        if (slidesClawSubsystem.atGoal()) {
            slidesClawSubsystem.stop()
        } else {
            slidesClawSubsystem.operateSlides()
        }
    }
}