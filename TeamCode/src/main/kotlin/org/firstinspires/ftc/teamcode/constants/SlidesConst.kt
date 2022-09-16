package org.firstinspires.ftc.teamcode.constants

class SlidesConst() {
    enum class SlidesPosition(val ticks: Long) {
        HIGH(100),
        MIDDLE(100),
        LOW(100),
        GROUND(100),
    }

    enum class SlidesPID(val coeff: Double) {
        P(0.0),
        I(0.0),
        D(0.0),
    }

    // TODO: Tune Tick and PID Constants
}