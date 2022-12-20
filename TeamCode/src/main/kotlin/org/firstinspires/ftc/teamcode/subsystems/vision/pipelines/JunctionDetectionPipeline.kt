package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline


class JunctionDetectionPipeline : OpenCvPipeline() {
    private var mat = Mat()
    private var contours: List<MatOfPoint> = ArrayList()

    private val submat = Rect(Point(0.0, 190.0), Point(639.0, 290.0))
    private val size = Size(19.0, 19.0)
    private val lowBound = Scalar(11.0, 115.0, 55.0)
    private val highBound = Scalar(34.0, 255.0, 255.0)
    private val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(17.0, 17.0))
    private val red = Scalar(255.0, 0.0, 0.0)

    override fun processFrame(input: Mat?): Mat {
        Imgproc.GaussianBlur(input, mat, size, 0.0)
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HLS)
        Core.inRange(mat, lowBound, highBound, mat)
        Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_CLOSE, kernel)
        Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_OPEN, kernel)
        Imgproc.findContours(mat, contours, null, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.rectangle(mat, submat, red)

        return mat
    }
}