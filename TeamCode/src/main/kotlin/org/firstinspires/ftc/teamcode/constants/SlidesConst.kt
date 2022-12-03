package org.firstinspires.ftc.teamcode.constants

class SlidesConst() {
    enum class SlidesPosition(val ticks: Double) {
        HIGH(1000.0),
        MIDDLE(700.0),
        LOW(420.0),
        INTAKE(150.0),
        GROUND(0.0)
    }

    enum class SlidesPID(val coeff: Double) {
        P(0.02),
        I(0.01),
        D(0.0),
        G(0.2)
    }
}