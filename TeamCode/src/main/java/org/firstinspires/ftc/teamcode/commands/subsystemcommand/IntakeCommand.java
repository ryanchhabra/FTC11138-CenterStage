package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(IntakeSubsystem.IntakeState state) {
        super(
                () -> Attachments.getInstance().intakeSubsystem.updateIntakeState(state)
        );
    }
}
