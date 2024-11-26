package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;

public class LiftTeleopCommand extends InstantCommand {
    public LiftTeleopCommand(int target) {
        super(
                () -> Attachments.getInstance().liftSubsystem.setTargetPosition(target)
        );
    }
}
