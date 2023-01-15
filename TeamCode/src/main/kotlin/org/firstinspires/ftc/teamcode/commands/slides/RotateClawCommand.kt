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
        when (targetPos) {
            ClawPositions.LEFT ->
                if (subsystem.goal == SlidesConst.SlidesPosition.GROUND)
                    return
                else
                    subsystem.rotateLeft()
            ClawPositions.RIGHT ->
                if (subsystem.goal == SlidesConst.SlidesPosition.GROUND)
                    return
                else
                    subsystem.rotateRight()
            ClawPositions.MIDDLE ->
                subsystem.rotateMid()
            ClawPositions.NORMAL ->
                if(subsystem.goal == SlidesConst.SlidesPosition.INTAKE)
                    subsystem.rotateNormal()
                else
                    return
        }
    }

    override fun isFinished(): Boolean = true
}