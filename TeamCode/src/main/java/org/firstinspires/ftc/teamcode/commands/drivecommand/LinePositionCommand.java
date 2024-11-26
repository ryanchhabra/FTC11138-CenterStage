package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Attachments;

public class LinePositionCommand extends CommandBase {

    public Pose2d startPose;
    public Pose2d targetPose;

    private Attachments robot = Attachments.getInstance();

    public LinePositionCommand(Pose2d start, Pose2d target) {
        this.startPose = start;
        this.targetPose = target;

        robot.followTrajectoryAsync(
                robot.trajectoryBuilder(startPose)
                        .lineToSplineHeading(targetPose)
                        .build()
        );
    }

    @Override
    public void execute() {
        robot.update();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !robot.isBusy();
    }
}
