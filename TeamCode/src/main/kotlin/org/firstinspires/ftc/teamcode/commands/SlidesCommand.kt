package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

class SlidesCommand(private val subsystem: SlidesSubsystem, private val slidesPosition: SlidesConst.SlidesPosition) : CommandBase() {
    init { addRequirements(subsystem) }

    override fun initialize() {
        subsystem.setGoal(slidesPosition.ticks)
    }

    override fun execute() {
        subsystem.operateSlides()
    }

    override fun isFinished(): Boolean {
        return subsystem.atGoal()
    }

    override fun end(interrupted: Boolean) = subsystem.stop()
}