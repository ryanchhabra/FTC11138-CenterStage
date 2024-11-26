package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.HangCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.ArmDropCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.DroneFireCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.HangLowerCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.HangRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftHeightChangeCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftHighCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.LiftTeleopCommand;
import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;


@TeleOp(name = "Solo")
public class CenterStageTeleOp_Solo extends CommandOpMode {

    private final Attachments robot = Attachments.getInstance();
    private final CommandScheduler cs = CommandScheduler.getInstance();
    private GamepadEx g1;

    TrackingWheelLocalizer localizer;
    Pose2d currentPose;
    double heading;
    double fieldCentricOffset;

    boolean lastLiftChangeJoystickUp;
    boolean lastLiftChangeJoystickDown;

    @Override
    public void initialize() {

        g1 = new GamepadEx(gamepad1);

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

        cs.schedule(new ArmIntakeCommand());

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        () -> cs.schedule(new ClawToggleCommand(robot, IntakeSubsystem.ClawSide.FRONT))
                );
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> cs.schedule(new ClawToggleCommand(robot, IntakeSubsystem.ClawSide.BACK))
                );

        g1.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(() -> cs.schedule(
                        new ClawCommand(IntakeSubsystem.ClawState.OPEN, IntakeSubsystem.ClawSide.BOTH)
                                .alongWith(new InstantCommand(Globals::startIntaking))
                ));
        g1.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> cs.schedule(
                        new ClawCommand(IntakeSubsystem.ClawState.CLOSED, IntakeSubsystem.ClawSide.BOTH)
                                .alongWith(new InstantCommand(Globals::stopIntaking))
                ));


        g1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> cs.schedule(new HangCommand(HangSubsystem.HangState.HIGH)));
        g1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> cs.schedule(
                        new ConditionalCommand(
                                new HangCommand(HangSubsystem.HangState.HANG),
                                new HangCommand(HangSubsystem.HangState.LOW),
                                () -> robot.hangSubsystem.getHangState() == HangSubsystem.HangState.HIGH
                        )
                ));


        g1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> cs.schedule(
                        new ParallelCommandGroup(
                                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, IntakeSubsystem.ClawSide.BOTH),
                                new ArmDropCommand(Constants.liftHigh, Constants.clawArmTeleDrop)
                                        .alongWith(new InstantCommand(Globals::startScoring)),
                                new LiftHighCommand()
                        )
                ));
        g1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> cs.schedule(
                        new ParallelCommandGroup(
                                new LiftTeleopCommand(Constants.liftMin),
                                new ArmIntakeCommand()
                                        .alongWith(new InstantCommand(Globals::startIntaking))
                        ).andThen(new ClawCommand(IntakeSubsystem.ClawState.OPEN, IntakeSubsystem.ClawSide.BOTH))
                ));


        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> cs.schedule(new LiftHighCommand()));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
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

        boolean liftChangeJoystickUp = g1.getButton(GamepadKeys.Button.DPAD_UP);
        boolean liftChangeJoystickDown = g1.getButton(GamepadKeys.Button.DPAD_DOWN);

        if (liftChangeJoystickUp && !lastLiftChangeJoystickUp) {
            cs.schedule(new LiftHeightChangeCommand(1));
        } else if (liftChangeJoystickDown && !lastLiftChangeJoystickDown) {
            cs.schedule(new LiftHeightChangeCommand(-1));
        }

        lastLiftChangeJoystickUp = liftChangeJoystickUp;
        lastLiftChangeJoystickDown = liftChangeJoystickDown;



        double fingerX = 0, fingerY = 0;

        if (gamepad1.touchpad_finger_1) {
            fingerX = gamepad1.touchpad_finger_1_x;
            fingerY = gamepad1.touchpad_finger_1_y;
        } else if (gamepad1.touchpad_finger_2) {
            fingerX = gamepad1.touchpad_finger_2_x;
            fingerY = gamepad1.touchpad_finger_2_y;
        }

        if (fingerX > 0 && gamepad1.touchpad) {
            cs.schedule(
                    new ClawCommand(IntakeSubsystem.ClawState.CLOSED, IntakeSubsystem.ClawSide.BOTH)
                            .alongWith(new InstantCommand(Globals::stopIntaking))
            );
        }
        if (fingerX < 0 && gamepad1.touchpad) {
            cs.schedule(
                    new ClawCommand(IntakeSubsystem.ClawState.OPEN, IntakeSubsystem.ClawSide.BOTH)
                            .alongWith(new InstantCommand(Globals::startIntaking))
            );
        }



        if (g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
            cs.schedule(new IntakeCommand(IntakeSubsystem.IntakeState.IN));
        }
        if (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            cs.schedule(new IntakeCommand(IntakeSubsystem.IntakeState.OUT));
        }
        if (g1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            cs.schedule(new IntakeCommand(IntakeSubsystem.IntakeState.STOP));
        }

        if (Globals.IS_SCORING && !Globals.IS_INTAKING && robot.armSubsystem.getArmState() == ArmSubsystem.ArmState.DROP) {
            cs.schedule(new IntakeCommand(IntakeSubsystem.IntakeState.STOP));
        } else if (Globals.IS_INTAKING && !Globals.IS_SCORING && robot.armSubsystem.getArmState() == ArmSubsystem.ArmState.INTAKE) {
            cs.schedule(new IntakeCommand(IntakeSubsystem.IntakeState.IN));
        } else if (!Globals.IS_INTAKING && !Globals.IS_SCORING) {
            cs.schedule(new IntakeCommand(IntakeSubsystem.IntakeState.OUT));
        } else {
            cs.schedule(new IntakeCommand(IntakeSubsystem.IntakeState.STOP));
        }


        telemetry.addData("SCORING", Globals.IS_SCORING);
        telemetry.addData("INTAKING", Globals.IS_INTAKING);

        telemetry.addLine();

        telemetry.addData("ChangeJoyUp", liftChangeJoystickUp);
        telemetry.addData("ChangeJoyDown", liftChangeJoystickDown);
        telemetry.addData("ChangeJoyUpLast", lastLiftChangeJoystickUp);
        telemetry.addData("ChangeJoyDownLast", lastLiftChangeJoystickDown);
        telemetry.addData("Lift Height", robot.liftSubsystem.getCurrentHeight());

        telemetry.update();

    }
}
