package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines

import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class JunctionDetectionPipeline : OpenCvPipeline() {
    private val gray = Mat()
    private val submat = Rect(Point(0.0, 190.0), Point(639.0, 290.0))

    override fun processFrame(input: Mat?): Mat {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2HSV)
        Imgproc.rectangle(gray, submat, Scalar(255.0, 0.0, 0.0))

        return gray
    }

}