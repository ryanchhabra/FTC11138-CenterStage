package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;

public class RaiseSystemCommand extends SequentialCommandGroup {
    public RaiseSystemCommand(int lift, double arm) {
        super(
                new ArmCommand(arm),
                new WaitCommand(300),
                new WristCommand(Constants.turnClaw180 - (Constants.turnClawValPerDegree * ((Constants.clawArmHigh - arm) / Constants.clawArmValPerDegree))),
                new LiftCommand(1, lift)
        );
    }

}
