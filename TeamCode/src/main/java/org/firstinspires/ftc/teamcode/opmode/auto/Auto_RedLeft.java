package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name="Auto_RedAudience", group="Linear Opmode", preselectTeleOp = "TeleOp")
public class Auto_RedLeft extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Globals.Alliance.RED;
        Globals.SIDE  = Globals.Side.AUDIENCE;

        robot.initialize(hardwareMap, telemetry);

        while (!isStarted()) {
            robot.getPropLocation();
            telemetry.addLine("Location: " + Globals.PROP_LOCATION.toString());
            telemetry.update();
        }
        robot.getPropLocation();
        telemetry.addLine("Final Location: " + Globals.PROP_LOCATION.toString());
        telemetry.update();



        PoseStorage.currentPose = robot.getPoseEstimate();

    }
}
