package org.firstinspires.ftc.teamcode.commands.slides

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem

class SlidesCommand(
    private val subsystem: SlidesClawSubsystem,
    private val targetPos: SlidesConst.SlidesPosition
) : CommandBase() {

    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        subsystem.goal = targetPos
    }

    override fun isFinished(): Boolean {
        return targetPos == subsystem.goal
    }
}