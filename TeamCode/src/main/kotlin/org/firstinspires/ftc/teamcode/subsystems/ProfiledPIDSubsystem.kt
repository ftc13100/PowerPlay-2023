package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.ProfiledPIDCommand
import com.arcrobotics.ftclib.command.SubsystemBase

abstract class ProfiledPIDSubsystem: SubsystemBase() {
    abstract val controller: Unit
}