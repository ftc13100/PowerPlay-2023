package org.firstinspires.ftc.teamcode.commands.claw

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.constants.SlidesConst.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem
import java.util.function.Supplier

class RotateClawCommand(
    private val subsystem: SlidesClawSubsystem,
    private val targetPos: ClawPositions,
    private val direction: Supplier<Pose2d>
): CommandBase() {
    override fun initialize() {
        val (_, _, heading) = direction.get()

        if (subsystem.goal != SlidesConst.SlidesPosition.GROUND || targetPos == ClawPositions.MIDDLE)
            when (targetPos) {
                ClawPositions.LEFT -> if (heading < Math.toRadians(180.0)) {
                    subsystem.rotateLeft()
                } else {
                    subsystem.rotateRight()
                }
                ClawPositions.RIGHT -> if (heading < Math.toRadians(180.0)) {
                    subsystem.rotateRight()
                } else {
                    subsystem.rotateLeft()
                }
                ClawPositions.MIDDLE -> subsystem.rotateMid()
                ClawPositions.NORMAL -> subsystem.rotateNormal()
            }
    }

    override fun isFinished(): Boolean = true
}