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
        drive.drive(adjustedInput(leftX, zoneVal), adjustedInput(leftY, zoneVal), rightX.asDouble)
    }

    private fun adjustedInput(input: DoubleSupplier, value: Double): Double {
        val inputVal = input.asDouble
        return when {
            abs(inputVal) < value -> 0.0
            else -> sign(inputVal) * (abs(inputVal) - value)
        }
    }
}