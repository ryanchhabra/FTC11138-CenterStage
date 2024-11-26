package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftHighCommand extends InstantCommand {
    public LiftHighCommand() {
        super(
                () -> new LiftTeleopCommand(Attachments.getInstance().liftSubsystem.getCurrentHeight())
        );
    }
}
