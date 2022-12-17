package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.PipelineParent
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraRotation

class VisionSubsystem<T : Map<String, PipelineParent<*>>>(
    private val camera: OpenCvCamera,
    private val pipelines: T,
    private val telemetry: Telemetry,
) : SubsystemBase() {

    private var currentPipeline: PipelineParent<*>

    init {
        currentPipeline = pipelines["April Tag"] ?: throw Error("Pipeline is null or does not exist!")
        camera.setPipeline(pipelines["April Tag"])

        camera.openCameraDeviceAsync(
            object: AsyncCameraOpenListener {
                override fun onOpened() {
                    telemetry.addData("Camera Initialized", true)
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
                }

                override fun onError(errorCode: Int) {
                    telemetry.addData("Camera Error", errorCode)
                    telemetry.update()
                }
            }
        )
    }

    fun switchPipeline(key: String) {
        camera.setPipeline(pipelines[key] ?: throw Error("Failed to switch pipeline!"))
    }

    fun getLatestResults() : List<*> {
        telemetry.addData("Latest Results", "Sent")
        return currentPipeline.getLatestResults()
    }
}

