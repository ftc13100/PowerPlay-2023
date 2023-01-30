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
        I(0.004),
        D(0.0006)
    }

    enum class ClawPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        NORMAL
    }

    enum class ClawState {
        OPEN,
        CLOSE
    }

    enum class SlidesProfile(val coeff: Double) {
        S(0.006),
        V(0.000467),
        G(0.05),
        A(0.001),
    }

    enum class SlidesConstraints(val value: Double) {
        MAX_VELOCITY(2140.0),
        MAX_ACCELERATION(2140.0)
    }
}