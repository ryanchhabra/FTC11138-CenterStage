package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.DroneCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DroneSubsystem;

public class DroneFireCommand extends InstantCommand {
    public DroneFireCommand() {
        super(
                () -> new DroneCommand(DroneSubsystem.DroneState.FIRED)
        );
    }
}
