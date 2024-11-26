package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class ClawCommand extends InstantCommand {
    public ClawCommand(IntakeSubsystem.ClawState state, IntakeSubsystem.ClawSide side) {
        super(
                () -> Attachments.getInstance().intakeSubsystem.updateClawState(state, side)
        );
    }
}
