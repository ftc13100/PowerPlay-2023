package org.firstinspires.ftc.teamcode.commands.slides

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem
import java.util.function.DoubleSupplier

class HeightCommand(
    private val subsystem: SlidesSubsystem,
    private val increase: DoubleSupplier
) : CommandBase() {

    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        subsystem.increaseTargetPosition(increase.asDouble * 10)
        subsystem.operateSlides()
    }

    override fun isFinished(): Boolean = subsystem.atTargetPosition() && increase.asDouble == 0.0

    override fun end(interrupted: Boolean) = subsystem.stall()
}