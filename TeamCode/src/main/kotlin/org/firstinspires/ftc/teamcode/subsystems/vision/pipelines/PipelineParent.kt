package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines

import org.openftc.easyopencv.OpenCvPipeline

sealed class PipelineParent<T> : OpenCvPipeline() {
    abstract fun getLatestResults() : List<T>
}