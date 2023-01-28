package org.firstinspires.ftc.teamcode.commands.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.pow

class DriveCommand(
    private val drive: DriveSubsystem,
    private val leftX: DoubleSupplier,
    private val leftY: DoubleSupplier,
    private val rightX: DoubleSupplier,
    private val zoneVal: Double,
) : CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.drive(
            leftY = -adjustedInput(leftY),
            leftX = adjustedInput(leftX),
            rightX = rightX.asDouble
        )
    }

    private fun adjustedInput(input: DoubleSupplier): Double = input.asDouble.pow(5)
}