package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.HangCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.HangSubsystem;

public class HangLowerCommand extends InstantCommand {
    public HangLowerCommand() {
        super(
                () -> new HangCommand(HangSubsystem.HangState.HANG)
        );
    }
}
