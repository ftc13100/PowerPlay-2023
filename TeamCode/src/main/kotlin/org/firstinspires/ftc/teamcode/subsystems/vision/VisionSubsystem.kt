package org.firstinspires.ftc.teamcode.subsystems.vision

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.OpenCvResultsPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

class VisionSubsystem<T>(
    webcam: WebcamName,
    cameraMonitorViewId: Int,
    private val pipeline: OpenCvResultsPipeline<T>,
    private val telemetry: Telemetry
) : SubsystemBase() {
    private val camera: OpenCvCamera

    init {
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId)

        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {}
        })

        camera.setPipeline(pipeline)
    }

    fun getLatestResults() = pipeline.getLatestResults()
}