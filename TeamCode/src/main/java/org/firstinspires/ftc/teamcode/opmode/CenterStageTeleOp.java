package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.teleop.ArmDropCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.DroneFireCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftHeightChangeCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftTeleopCommand;
import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.HangLowerCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.HangRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftHighCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftPowerCommand;
import org.firstinspires.ftc.teamcode.hardware.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;

@TeleOp(name = "Duo")
public class CenterStageTeleOp extends CommandOpMode {

    private final Attachments robot = Attachments.getInstance();
    private final CommandScheduler cs = CommandScheduler.getInstance();
    private GamepadEx g1;
    private GamepadEx g2;

    TrackingWheelLocalizer localizer;
    Pose2d currentPose;
    double heading;
    double fieldCentricOffset;

    boolean lastLiftChangeJoystickUp;
    boolean lastLiftChangeJoystickDown;

    @Override
    public void initialize() {

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        Globals.IS_AUTO = false;

        localizer = new TrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());
        localizer.setPoseEstimate(PoseStorage.currentPose);

        robot.initialize(hardwareMap, telemetry);

        Globals.stopIntaking();
        Globals.stopScoring();

        switch (Globals.ALLIANCE) {
            case BLUE:
                fieldCentricOffset = Math.toRadians(-90);
                break;
            case RED:
                fieldCentricOffset = Math.toRadians(90);
                break;
        }

        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawToggleCommand(robot, IntakeSubsystem.ClawSide.FRONT));
        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawToggleCommand(robot, IntakeSubsystem.ClawSide.BACK));

        g2.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(() -> cs.schedule(
                        new ClawCommand(IntakeSubsystem.ClawState.OPEN, IntakeSubsystem.ClawSide.BOTH)
                            .alongWith(new InstantCommand(Globals::startIntaking))
                ));
        g2.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> cs.schedule(
                        new ClawCommand(IntakeSubsystem.ClawState.CLOSED, IntakeSubsystem.ClawSide.BOTH)
                                .alongWith(new InstantCommand(Globals::stopIntaking))
                ));


        g2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> cs.schedule(new HangRaiseCommand()));
        g2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> cs.schedule(new HangLowerCommand()));


        g2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> cs.schedule(
                        new ParallelCommandGroup(
                                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, IntakeSubsystem.ClawSide.BOTH),
                                new ArmDropCommand(Constants.liftHigh, Constants.clawArmTeleDrop)
                                        .alongWith(new InstantCommand(Globals::startScoring)),
                                new LiftHighCommand()
                        )
                ));
        g2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> cs.schedule(
                        new ParallelCommandGroup(
                                new LiftTeleopCommand(Constants.liftMin),
                                new ArmIntakeCommand()
                                        .alongWith(new InstantCommand(Globals::startIntaking))
                        ).alongWith(new ClawCommand(IntakeSubsystem.ClawState.OPEN, IntakeSubsystem.ClawSide.BOTH))
                ));


        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> cs.schedule(new LiftHighCommand()));
        g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> cs.schedule(new LiftDownCommand()));


        g1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> cs.schedule(
                        new DroneFireCommand()
                                .alongWith(new InstantCommand(() -> gamepad1.rumble(1000)))
                                .alongWith(new InstantCommand(() -> gamepad2.rumble(1000)))
                ));


        g1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(
                        new InstantCommand(() -> localizer.setPoseEstimate(new Pose2d(0, 0, 0)))
                                .alongWith(new InstantCommand(() -> gamepad1.rumble(250)))
                );

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new InstantCommand(() -> Globals.LIMITS = !Globals.LIMITS)
                                .alongWith(new InstantCommand(() -> gamepad1.rumble(150)))
                );

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


        double lx = 0;
        double ly = 0;
        double speedMultiplier = Constants.moveSpeed;
        double rotationMultiplier = Constants.rotSpeed;

        // D-pad
        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = 0.6;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = 0.6;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = 0.6;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = 0.6;
        }

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        // Drive
        if (lx != 0 || ly != 0) {
            robot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);
        } else {
            robot.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
        }

        double liftJoy = -gamepad2.left_stick_y;
        cs.schedule(new LiftPowerCommand(liftJoy));



        boolean liftChangeJoystickUp = gamepad2.right_stick_y < -0.5;
        boolean liftChangeJoystickDown = gamepad2.right_stick_y > 0.5;

        if (liftChangeJoystickUp && !lastLiftChangeJoystickUp) {
            cs.schedule(new LiftHeightChangeCommand(1));
        } else if (liftChangeJoystickDown && !lastLiftChangeJoystickDown) {
            cs.schedule(new LiftHeightChangeCommand(-1));
        }

        lastLiftChangeJoystickUp = liftChangeJoystickUp;
        lastLiftChangeJoystickDown = liftChangeJoystickDown;



        if (g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) new IntakeCommand(IntakeSubsystem.IntakeState.IN);
        if (g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) new IntakeCommand(IntakeSubsystem.IntakeState.OUT);
        if (g2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) new IntakeCommand(IntakeSubsystem.IntakeState.STOP);


        if (Globals.IS_SCORING && !Globals.IS_INTAKING) {
            new IntakeCommand(IntakeSubsystem.IntakeState.STOP);
        } else if (Globals.IS_INTAKING && !Globals.IS_SCORING) {
            new IntakeCommand(IntakeSubsystem.IntakeState.IN);
        } else {
            new IntakeCommand(IntakeSubsystem.IntakeState.OUT);
        }

        telemetry.addData("SCORING", Globals.IS_SCORING);
        telemetry.addData("INTAKING", Globals.IS_INTAKING);

    }
}
