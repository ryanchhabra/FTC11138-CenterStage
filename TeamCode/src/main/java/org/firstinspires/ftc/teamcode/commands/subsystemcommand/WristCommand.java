package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;

public class WristCommand extends InstantCommand {
    public WristCommand(double target) {
        super(
                () -> Attachments.getInstance().armSubsystem.setWristServo(target)
        );
    }
}
