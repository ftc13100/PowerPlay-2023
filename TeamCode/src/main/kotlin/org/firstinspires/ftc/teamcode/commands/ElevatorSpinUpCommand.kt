package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.OpenElevatorSubsystem

class ElevatorSpinUpCommand(private val subsystem: OpenElevatorSubsystem): CommandBase() {
    override fun execute() = subsystem.spinUp()

    override fun end(interrupted: Boolean) = subsystem.stopSpin()

}