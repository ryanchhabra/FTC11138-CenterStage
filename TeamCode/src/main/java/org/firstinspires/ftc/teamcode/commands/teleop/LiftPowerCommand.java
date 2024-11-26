package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;

public class LiftPowerCommand extends InstantCommand {
    public LiftPowerCommand(double power) {
        super(
                () -> Attachments.getInstance().liftSubsystem.setPower(power)
        );
    }
}
