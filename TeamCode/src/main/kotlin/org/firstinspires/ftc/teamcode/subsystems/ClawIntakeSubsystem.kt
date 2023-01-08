package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

// For the sake of consistency, kind of useless, but allows for adaptability later on
class ClawIntakeSubsystem(
    private val clawServo: Servo,
    private val rotationServo: Servo
) : SubsystemBase() {

    fun openClaw() { clawServo.position = 1.0 }

    fun closeClaw() { clawServo.position = 0.0 }

    fun rotateLeft() { rotationServo.position = 0.0 }

    fun rotateMid() { rotationServo.position = 0.5 }

    fun rotateRight() { rotationServo.position = 1.0 }
}
