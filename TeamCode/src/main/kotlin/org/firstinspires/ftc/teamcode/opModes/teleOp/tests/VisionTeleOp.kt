package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.JunctionDetectionPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@TeleOp(name = "Junction Vision Test", group = "Tests")
class VisionTeleOp : LinearOpMode() {
    @SuppressLint("DiscouragedApi")
    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val monitorId: Int = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val webcamName: WebcamName = hardwareMap.get(WebcamName::class.java, DeviceConfig.VISION_CAMERA.deviceName)
        val camera: OpenCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, monitorId)
        val pipeline = JunctionDetectionPipeline()

        camera.setPipeline(pipeline)

        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
            }
        })

        dash.startCameraStream(camera, 0.0)

        waitForStart()

        while (opModeIsActive()) {
            continue
        }
    }
}