package org.firstinspires.ftc.teamcode.constants

class SlidesConst {
    enum class SlidesPosition(val ticks: Double) {
        HIGH(2973.0),
        MIDDLE(2048.0),
        LOW(1074.0),
        INTAKE(250.0),
        GROUND(0.0)
    }

    enum class SlidesPID(val coeff: Double) {
        P(0.02),
        I(0.0),
        D(0.0001)
    }

    enum class ClawPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        NORMAL
    }

    enum class SlidesProfile(val coeff: Double) {
        S(0.03)
    }

    enum class SlidesConstraints(val value: Double) {
        MAX_VELOCITY(2140.0),
        MAX_ACCELERATION(2140.0)
    }
}