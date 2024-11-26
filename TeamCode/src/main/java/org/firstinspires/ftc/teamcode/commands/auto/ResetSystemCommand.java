package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;

public class ResetSystemCommand extends SequentialCommandGroup {
    public ResetSystemCommand() {
        super(
                new LiftCommand(1, Constants.liftMin),
                new ArmCommand(Constants.clawArmLow),
                new WaitCommand(1000),
                new WristCommand(Constants.turnClawDown),
                new WaitCommand(300),
                new ArmCommand(Constants.clawArmDown)
        );
    }
}
