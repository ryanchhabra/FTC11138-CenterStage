package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DroneSubsystem;

public class DroneCommand extends InstantCommand {
    public DroneCommand(DroneSubsystem.DroneState state) {
        super(
                () -> Attachments.getInstance().droneSubsystem.updateState(state)
        );
    }
}
