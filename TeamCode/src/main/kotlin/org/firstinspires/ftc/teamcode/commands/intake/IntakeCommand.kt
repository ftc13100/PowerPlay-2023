package org.firstinspires.ftc.teamcode.commands.intake

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem

class IntakeCommand(
    private val subsystem: IntakeSubsystem,
    private val intake: Boolean
) : CommandBase() {

    override fun execute() = if (intake) subsystem.intake() else subsystem.outtake()

    override fun end(interrupted: Boolean) = subsystem.stop()
}