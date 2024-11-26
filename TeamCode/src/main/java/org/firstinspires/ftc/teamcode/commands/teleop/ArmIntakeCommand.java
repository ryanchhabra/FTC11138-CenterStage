package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ArmIntakeCommand extends SequentialCommandGroup {

    private final Attachments robot = Attachments.getInstance();

    public ArmIntakeCommand() {
        super(
                new ArmStateCommand(ArmSubsystem.ArmState.INTAKE),
                new ArmCommand(Constants.clawArmLow),
                new WaitCommand(1000),
                new WristCommand(Constants.turnClawDown),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, IntakeSubsystem.ClawSide.BOTH),
                new WaitCommand(300),
                new ArmCommand(Constants.clawArmDown)
        );
    }
}
