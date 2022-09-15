package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.arcrobotics.ftclib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem

class IntakeCommand(
    private val subsystem: IntakeSubsystem,
    private val intake: Boolean
) : CommandBase() {
    init { addRequirements(subsystem as Subsystem) }

    override fun execute() = if (intake) subsystem.intake() else subsystem.outtake()

    override fun end(interrupted: Boolean) = subsystem.stop()
}