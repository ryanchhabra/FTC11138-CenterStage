package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.DroneCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DroneSubsystem;

public class DroneArmCommand extends CommandBase {
    public DroneArmCommand() {
        new DroneCommand(DroneSubsystem.DroneState.ARMED);
    }
}
