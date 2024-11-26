package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.subsystems.AutonomousClawSubsystem;

public class AutonomousClawCommand extends InstantCommand {
    public AutonomousClawCommand(AutonomousClawSubsystem.AutoClawState state) {
        super(
                () -> Attachments.getInstance().autonomousClawSubsystem.updateState(state)
        );
    }
}
