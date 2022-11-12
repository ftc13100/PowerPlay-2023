package org.firstinspires.ftc.teamcode.constants

class SlidesConst() {
    enum class SlidesPosition(val ticks: Double) {
        HIGH(100.0),
        MIDDLE(100.0),
        LOW(100.0),
        GROUND(100.0),
    }

    enum class SlidesPID(val coeff: Double) {
        P(0.0),
        I(0.0),
        D(0.0),
        F(0.0),
    }

    enum class SlidesFeedforward(val coeff: Double) {
        Ks(0.0),
        Kg(0.0),
        Kv(0.0),
        Ka(0.0),
    }
    // TODO: Tune Tick and PID Constants
}