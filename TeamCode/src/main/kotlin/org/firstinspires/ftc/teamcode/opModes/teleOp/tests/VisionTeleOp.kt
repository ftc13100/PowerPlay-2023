package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.constants.AprilTagCamera.*
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.AprilTagDetectionPipeline

@TeleOp(name = "Vision Test", group = "Test")
class VisionTeleOp : LinearOpMode() {

    // Telemetry
    init {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    // Hardware
    private val webcamName = hardwareMap.get(WebcamName::class.java, DeviceConfig.VISION_CAMERA.deviceName)
    private val cameraMonitorId =
        hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)

    // Subsystems
    private val pipeline = AprilTagDetectionPipeline(TAGSIZE.value, FX.value, FY.value, CX.value, CY.value)
    private val vision = VisionSubsystem(webcamName, cameraMonitorId, pipeline, telemetry)

    override fun runOpMode() {
        waitForStart()

        vision.getLatestResults().forEach { result -> telemetry.addData("ID", result.id) }
    }
}