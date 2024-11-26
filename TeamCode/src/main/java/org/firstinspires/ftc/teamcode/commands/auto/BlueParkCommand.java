package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.commands.drivecommand.LinePositionCommand;

public class BlueParkCommand extends SequentialCommandGroup {
    public BlueParkCommand(Attachments robot, double parkH) {
        super(
                new ConditionalCommand(
                        new LinePositionCommand(robot.getPoseEstimate(), new Pose2d(48, 12, parkH))
                                .andThen(new LinePositionCommand(robot.getPoseEstimate(), new Pose2d(60, 12, parkH))),
                        new LinePositionCommand(robot.getPoseEstimate(), new Pose2d(48, 60, parkH))
                                .andThen(new LinePositionCommand(robot.getPoseEstimate(), new Pose2d(60, 60, parkH))),
                        () -> Globals.PARK_SIDE == Globals.ParkSide.MIDDLE
                )
        );
    }
}
