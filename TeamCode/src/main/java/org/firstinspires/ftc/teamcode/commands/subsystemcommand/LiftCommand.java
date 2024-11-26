package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;

public class LiftCommand extends InstantCommand {
    public LiftCommand(double power, int target) {
        super(
                () -> Attachments.getInstance().liftSubsystem.setLiftPosition(1.0, target)
        );
    }
}
