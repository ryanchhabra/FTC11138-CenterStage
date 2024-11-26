package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftDownCommand extends InstantCommand {
    public LiftDownCommand() {
        super(
                () -> new LiftTeleopCommand(Constants.liftMin)
        );
    }
}
