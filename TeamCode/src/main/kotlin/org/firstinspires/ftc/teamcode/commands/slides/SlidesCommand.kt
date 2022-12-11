package org.firstinspires.ftc.teamcode.commands.slides

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

class SlidesCommand(
    private val subsystem: SlidesSubsystem,
    private val targetPos: SlidesConst.SlidesPosition
) : CommandBase() {

    init {
        addRequirements(subsystem)
    }

    override fun initialize() = subsystem.setTargetPosition(targetPos)

    override fun execute() = subsystem.operateSlides()

    override fun isFinished(): Boolean {
        return if (targetPos == SlidesConst.SlidesPosition.GROUND) {
            subsystem.atTargetPosition() || subsystem.isPressed()
        } else {
            subsystem.atTargetPosition()
        }
    }

    override fun end(interrupted: Boolean) {
        if (targetPos == SlidesConst.SlidesPosition.GROUND) {
            subsystem.stop()
        } else {
            subsystem.stall()
        }
    }
}