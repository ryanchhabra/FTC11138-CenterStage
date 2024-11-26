package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Attachments;

public class SplinePositionCommand extends CommandBase {

    public Pose2d startPose;
    public Pose2d targetPose;

    private Attachments robot = Attachments.getInstance();

    public SplinePositionCommand(Pose2d start, Pose2d target, double st, double et) {
        this.startPose = start;
        this.targetPose = target;

        robot.followTrajectoryAsync(
                robot.trajectoryBuilder(startPose, st)
                        .splineToSplineHeading(targetPose, et)
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
