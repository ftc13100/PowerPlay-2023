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

    override fun initialize() { subsystem.goal = targetPos }

    override fun execute() {
        if (targetPos == SlidesConst.SlidesPosition.GROUND) {
            subsystem.rotateMid()
        }
        subsystem.operateSlides()
    }

    override fun isFinished(): Boolean {
        return if (targetPos == SlidesConst.SlidesPosition.GROUND) {
            subsystem.atGoal() || subsystem.isPressed()
        } else {
            subsystem.atGoal()
        }
    }

    override fun end(interrupted: Boolean) = subsystem.stop()
}