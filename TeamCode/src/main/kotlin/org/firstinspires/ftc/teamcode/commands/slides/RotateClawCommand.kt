package org.firstinspires.ftc.teamcode.commands.slides

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.constants.SlidesConst.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem

class RotateClawCommand(
    private val subsystem: SlidesClawSubsystem,
    private val targetPos: ClawPositions
): CommandBase() {
    override fun initialize() {
        if (subsystem.goal != SlidesConst.SlidesPosition.GROUND || targetPos == ClawPositions.MIDDLE) {
            when (targetPos) {
                ClawPositions.LEFT -> subsystem.rotateLeft()
                ClawPositions.RIGHT -> subsystem.rotateRight()
                ClawPositions.MIDDLE -> subsystem.rotateMid()
                ClawPositions.NORMAL -> subsystem.rotateNormal()
            }
        }
    }

    override fun isFinished(): Boolean = true
}