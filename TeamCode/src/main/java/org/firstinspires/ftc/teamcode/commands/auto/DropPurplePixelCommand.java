package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.AutonomousClawCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.AutonomousClawSubsystem;

public class DropPurplePixelCommand extends SequentialCommandGroup {
    public DropPurplePixelCommand() {
        super(
                new AutonomousClawCommand(AutonomousClawSubsystem.AutoClawState.RELEASE),
                new WaitCommand(200),
                new AutonomousClawCommand(AutonomousClawSubsystem.AutoClawState.HOLD)
        );
    }
}
