package org.firstinspires.ftc.teamcode.constants

class SlidesConst {
    enum class SlidesPosition(val ticks: Double) {
        HIGH(1000.0),
        MIDDLE(750.0),
        LOW(500.0),
        INTAKE(250.0),
        GROUND(0.0)
    }

    enum class SlidesPID(val coeff: Double) {
        P(0.02),
        I(0.01),
        D(0.0),
        G(0.2)
    }

    enum class SlidesProfile(val coeff: Double) {
        V(0.0),
        A(0.0),
        S(0.0)
    }

    enum class SlidesConstraints(val value: Double) {
        MAX_VELOCITY(2140.0),
        MAX_ACCELERATION(2140.0)
    }
}