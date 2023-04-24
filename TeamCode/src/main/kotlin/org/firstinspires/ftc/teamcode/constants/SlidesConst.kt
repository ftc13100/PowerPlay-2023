package org.firstinspires.ftc.teamcode.constants

class SlidesConst {
    enum class SlidesPosition(val ticks: Double) {
        HIGH(2850.0),
        MIDDLE(2000.0),
        LOW(1200.0),
        INTAKE(250.0),
        GROUND(0.0)
    }

    enum class SlidesPID(val coeff: Double) {
        P(0.02),
        I(0.0),
        D(0.0005)
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
        G(0.05),
    }

    enum class SlidesConstraints(val value: Double) {
        MAX_VELOCITY(2160.0),
        MAX_ACCELERATION(2179.9999882429843)
    }
}