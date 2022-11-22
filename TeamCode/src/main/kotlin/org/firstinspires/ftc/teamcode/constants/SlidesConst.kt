package org.firstinspires.ftc.teamcode.constants

class SlidesConst() {
    enum class SlidesPosition(val ticks: Double) {
        HIGH(930.0),
        MIDDLE(530.0),
        LOW(330.0),
        GROUND(0.0),
    }

    enum class SlidesPID(val coeff: Double) {
        P(0.008),
        I(0.0),
        D(0.0003),
        G(0.3)
    }

    enum class SlidesFeedforward(val coeff: Double) {
        Ks(0.0),
        Kg(0.0),
        Kv(0.0),
        Ka(0.0),
    }
    // TODO: Tune Tick and PID Constants
}