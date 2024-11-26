package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.auto.DroneArmCommand;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.auto.BlueParkCommand;
import org.firstinspires.ftc.teamcode.commands.auto.DropPurplePixelCommand;
import org.firstinspires.ftc.teamcode.commands.auto.RaiseSystemCommand;
import org.firstinspires.ftc.teamcode.commands.auto.ResetSystemCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.LinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

@Autonomous(name="Auto_BlueBackdrop", group="Linear Opmode", preselectTeleOp = "TeleOp")
public class Auto_BlueLeft extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Globals.Alliance.BLUE;
        Globals.SIDE  = Globals.Side.BACKDROP;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        while (!isStarted()) {
            robot.getPropLocation();
            telemetry.addLine("Location: " + Globals.PROP_LOCATION.toString());
            telemetry.update();
        }
        robot.getPropLocation();
        int propLocation = robot.propLocationToInt(Globals.PROP_LOCATION);
        telemetry.addLine("Final Location: " + Globals.PROP_LOCATION.toString());
        telemetry.update();

        robot.setPoseEstimate(PoseConstants.newBlueLeft.start);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new DroneArmCommand(),
                        new LinePositionCommand(PoseConstants.newBlueLeft.start, PoseConstants.newBlueLeft.backdrop[propLocation-1])
                                .alongWith(new WaitCommand(500)
                                        .andThen(new RaiseSystemCommand(Constants.liftDropAuto, Constants.clawArmAutoDrop))
                                ),
                        new ClawCommand(IntakeSubsystem.ClawState.OPEN, IntakeSubsystem.ClawSide.BOTH),
                        new LinePositionCommand(robot.getPoseEstimate(), PoseConstants.newBlueLeft.pixel[propLocation-1])
                                .alongWith(new ResetSystemCommand()),
                        new DropPurplePixelCommand(),
                        new BlueParkCommand(robot, Math.toRadians(0))
                )
        );

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            telemetry.addLine(robot.getPoseEstimate().toString());
            PoseStorage.currentPose = robot.getPoseEstimate();
        }

    }
}
