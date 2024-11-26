package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ArmSubsystem;

public class ArmStateCommand extends InstantCommand {
    public ArmStateCommand(ArmSubsystem.ArmState state) {
        super(
                () -> Attachments.getInstance().armSubsystem.updateArmState(state)
        );
    }
}
