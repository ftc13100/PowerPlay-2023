package org.firstinspires.ftc.teamcode.commands.slides

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem
import java.util.function.DoubleSupplier

class HeightCommand(
    private val subsystem: SlidesClawSubsystem,
    private val increase: DoubleSupplier
) : CommandBase() {

    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        subsystem.increaseTargetPosition(increase.asDouble * 10)
        subsystem.operateSlides()
    }

    override fun isFinished(): Boolean = subsystem.atGoal() && increase.asDouble == 0.0

    override fun end(interrupted: Boolean) = subsystem.stop()
}