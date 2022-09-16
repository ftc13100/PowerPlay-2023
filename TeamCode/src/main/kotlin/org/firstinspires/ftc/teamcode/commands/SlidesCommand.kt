package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

class SlidesCommand(private val subsystem: SlidesSubsystem, private val movementUp: Boolean) : CommandBase() {
    init { addRequirements(subsystem as Subsystem) }

    override fun execute() = if (movementUp) subsystem.slideUp() else subsystem.slideDown()

    override fun end(interrupted: Boolean) = subsystem.stop()
}