package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.util.Globals;

public class LiftHeightChangeCommand extends SequentialCommandGroup {
    public LiftHeightChangeCommand(int increment) {
        super(
                new InstantCommand(() -> Attachments.getInstance().liftSubsystem.changeCurrentHeight(increment))
//                new ConditionalCommand(
//                        new LiftTeleopCommand(Attachments.getInstance().liftSubsystem.getCurrentHeight()),
//                        new InstantCommand(),
//                        () -> Globals.IS_SCORING
//                )
        );
    }
}
