package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class ClawToggleCommand extends ConditionalCommand {
    public ClawToggleCommand(Attachments robot, IntakeSubsystem.ClawSide side) {
        super (
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, side),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, side),
                () -> (robot.intakeSubsystem.getClawState(side) == IntakeSubsystem.ClawState.CLOSED)
        );
    }
}
