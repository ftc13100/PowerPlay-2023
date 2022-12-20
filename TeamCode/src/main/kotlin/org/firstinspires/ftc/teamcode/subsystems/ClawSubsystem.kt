package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

// For the sake of consistency, kind of useless, but allows for adaptability later on
class ClawSubsystem(private val clawServo: Servo) : SubsystemBase() {
    fun open() { clawServo.position = 1.0 }

    fun close() { clawServo.position = 0.0 }
}
