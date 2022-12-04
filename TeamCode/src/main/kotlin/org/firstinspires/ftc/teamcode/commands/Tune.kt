package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.OpenElevatorSubsystem

class Tune(private val subsystem: OpenElevatorSubsystem): CommandBase() {
    override fun execute() = subsystem.spinUp()

    override fun isFinished() = subsystem.getPosition() > 900

    override fun end(interrupted: Boolean) = subsystem.stopSpin()

}