package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;

public class ArmCommand extends InstantCommand {
    public ArmCommand(double target) {
        super(
                () -> Attachments.getInstance().armSubsystem.setArmServo(target)
        );
    }
}
