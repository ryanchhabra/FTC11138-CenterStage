package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;

public class BaseOpMode extends CommandOpMode {

    private final Attachments robot = Attachments.getInstance();
    private final CommandScheduler cs = CommandScheduler.getInstance();
    private GamepadEx g1;

    TrackingWheelLocalizer localizer;
    Pose2d currentPose;
    double heading;
    double fieldCentricOffset;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;
        Globals.stopIntaking();
        Globals.stopScoring();

        localizer = new TrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());
        localizer.setPoseEstimate(PoseStorage.currentPose);

        g1 = new GamepadEx(gamepad1);

        robot.initialize(hardwareMap, telemetry);

        switch (Globals.ALLIANCE) {
            case BLUE:
                fieldCentricOffset = Math.toRadians(-90);
                break;
            case RED:
                fieldCentricOffset = Math.toRadians(90);
                break;
        }



    }

    @Override
    public void run() {

        cs.run();
        robot.periodic();

        localizer.update();
        currentPose = localizer.getPoseEstimate();
        heading = -currentPose.getHeading();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(heading + fieldCentricOffset);


        robot.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

    }
}
