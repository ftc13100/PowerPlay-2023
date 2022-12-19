package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines

import com.acmerobotics.dashboard.config.Config
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

@Config
class JunctionDetectionPipeline : OpenCvPipeline() {
    private val mat = Mat()
    private val submat = Rect(Point(0.0, 190.0), Point(639.0, 290.0))

    val highBound = Scalar(127.0, 220.0, 255.0)
    val lowBound = Scalar(5.0, 30.0, 220.0)
    override fun processFrame(input: Mat?): Mat {

        Imgproc.GaussianBlur(input, mat, Size(19.0, 19.0), 0.0);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HLS)
        Core.inRange(mat, lowBound, highBound, mat)
        Imgproc.rectangle(mat, submat, Scalar(255.0, 0.0, 0.0))

        return mat
    }
}