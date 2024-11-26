package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ArmDropCommand extends SequentialCommandGroup {

    private final Attachments robot = Attachments.getInstance();

    public ArmDropCommand(int lift, double arm) {
        super(
                new ArmStateCommand(ArmSubsystem.ArmState.DROP),
                new ArmCommand(arm),
                new WaitCommand(300),
                new WristCommand(Constants.turnClaw180 - (Constants.turnClawValPerDegree * ((Constants.clawArmHigh - arm) / Constants.clawArmValPerDegree))),
                new LiftTeleopCommand(lift)
        );
    }

}
