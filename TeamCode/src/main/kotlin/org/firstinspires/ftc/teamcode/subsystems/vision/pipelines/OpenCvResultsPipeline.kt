package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines

import org.openftc.easyopencv.OpenCvPipeline

abstract class OpenCvResultsPipeline<T> : OpenCvPipeline() {
    abstract fun getLatestResults(): List<T>
}