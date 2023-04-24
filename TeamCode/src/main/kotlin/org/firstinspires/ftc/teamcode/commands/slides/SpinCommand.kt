package org.firstinspires.ftc.teamcode.commands.slides

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem

class SpinCommand(private val subsystem: SlidesClawSubsystem, private val spinningUp: Boolean): CommandBase() {
    override fun execute() = if(spinningUp) subsystem.spinUp() else subsystem.spinDown()

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }
}