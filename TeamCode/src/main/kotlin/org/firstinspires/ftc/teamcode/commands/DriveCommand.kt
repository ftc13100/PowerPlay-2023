package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.sign

class DriveCommand(
    private val drive: DriveSubsystem,
    private val leftX: DoubleSupplier,
    private val leftY: DoubleSupplier,
    private val rightX: DoubleSupplier,
    private val zoneVal: Double,
) : CommandBase() {
    override fun execute() {
        drive.drive(adjustedInput(leftX), adjustedInput(leftY), rightX.asDouble)
    }

    private fun adjustedInput(input: DoubleSupplier): Double =
        if (abs(input.asDouble) < zoneVal) 0.0
        else sign(input.asDouble) * (abs(input.asDouble) - zoneVal)
}