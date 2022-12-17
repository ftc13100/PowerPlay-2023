/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines

import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.openftc.apriltag.AprilTagDetection
import org.openftc.apriltag.AprilTagDetectorJNI
import org.openftc.easyopencv.OpenCvPipeline

class AprilTagDetectionPipeline(// UNITS ARE METERS
    private val tagsize: Double,
    private val fx: Double,
    private val fy: Double,
    private val cx: Double,
    private val cy: Double
) : PipelineParent<AprilTagDetection>() {
    private var nativeApriltagPtr: Long
    private val gray = Mat()
    private var latestDetections = ArrayList<AprilTagDetection>()

    init {
        nativeApriltagPtr =
            AprilTagDetectorJNI.createApriltagDetector(
                AprilTagDetectorJNI.TagFamily.TAG_standard41h12.string, 3f, 3
            )
    }

    fun finalize() {
        if (nativeApriltagPtr != 0L) {
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr)
            nativeApriltagPtr = 0
        }
    }

    override fun processFrame(input: Mat): Mat {
        // Convert to grayscale
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY)

        // Run AprilTag
        latestDetections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
            nativeApriltagPtr, gray,
            tagsize, fx, fy, cx, cy
        )

        return input
    }

    override fun getLatestResults(): List<AprilTagDetection> {
        val results = ArrayList<AprilTagDetection>()
        results.addAll(latestDetections)
        return results
    }
}