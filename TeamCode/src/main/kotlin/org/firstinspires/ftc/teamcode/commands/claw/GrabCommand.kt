package org.firstinspires.ftc.teamcode.commands.claw

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem

class GrabCommand(
    private val shouldOpen: Boolean,
    private val subsystem: ClawIntakeSubsystem
) : CommandBase() {

    override fun execute() = if (shouldOpen) {
        subsystem.openClaw()
    } else {
        subsystem.closeClaw()
    }

    override fun isFinished(): Boolean {
        return true;
    }
}