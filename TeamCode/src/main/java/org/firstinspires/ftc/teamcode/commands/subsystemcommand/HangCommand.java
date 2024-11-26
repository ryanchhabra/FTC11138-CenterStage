package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.subsystems.HangSubsystem;

public class HangCommand extends InstantCommand {
    public HangCommand(HangSubsystem.HangState state) {
        super(
                () -> Attachments.getInstance().hangSubsystem.updateHangState(state)
        );
    }
}
