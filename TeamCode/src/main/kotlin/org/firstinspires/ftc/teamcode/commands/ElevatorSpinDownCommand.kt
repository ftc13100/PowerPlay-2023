package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.OpenElevatorSubsystem

class ElevatorSpinDownCommand(private val subsystem: OpenElevatorSubsystem): CommandBase() {
    override fun execute() = subsystem.spinDown()

    override fun isFinished(): Boolean {
        return subsystem.isPressed()
    }

    override fun end(interrupted: Boolean) = subsystem.stopSpin()
}