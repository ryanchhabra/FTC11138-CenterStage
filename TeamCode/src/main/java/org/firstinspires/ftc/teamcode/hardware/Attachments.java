package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.AutonomousClawSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.vision.Stack_VisionProcessor;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Attachments extends MecanumDrive {


    public Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    public WebcamName webcam;
    public VisionProcessor visionProcessor;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;
    public Stack_VisionProcessor stackProcessor;

    ArrayList<Subsystem> subsystems = new ArrayList<>();

    public IntakeSubsystem intakeSubsystem;
    public HangSubsystem hangSubsystem;
    public ArmSubsystem armSubsystem;
    public LiftSubsystem liftSubsystem;
    public AutonomousClawSubsystem autonomousClawSubsystem;
    public DroneSubsystem droneSubsystem;


    private static Attachments instance = null;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_) {

        if (Globals.ALLIANCE == null) Globals.ALLIANCE = Globals.Alliance.BLUE;

        // Random Stuff IDK
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize Roadrunner
        initializeRoadrunner(hardwareMap);

        telemetry.addLine("Roadrunner Initialized");




        intakeSubsystem = new IntakeSubsystem(hardwareMap, names.claw1Servo, names.claw2Servo, names.intake1, names.intake2, names.leds);
        hangSubsystem = new HangSubsystem(hardwareMap, names.hangMotor1, names.hangMotor2);
        armSubsystem = new ArmSubsystem(hardwareMap, names.clawArmServo, names.turnClawServo);
        liftSubsystem = new LiftSubsystem(hardwareMap, names.liftMotor);
        autonomousClawSubsystem = new AutonomousClawSubsystem(hardwareMap, names.pixelServo);
        droneSubsystem = new DroneSubsystem(hardwareMap, names.planeServo);

        subsystems.add(intakeSubsystem);
        subsystems.add(hangSubsystem);
        subsystems.add(armSubsystem);
        subsystems.add(liftSubsystem);
        subsystems.add(autonomousClawSubsystem);
        subsystems.add(droneSubsystem);



        // Camera
        webcam = hardwareMap.get(WebcamName.class, names.webcam);


        // Change Drive Motor Modes if not autonomous
        if (!Globals.IS_AUTO) {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            startCamera(hardwareMap);
        } else {
            startCamera(hardwareMap);
        }

    }

    public void periodic() {
        hangSubsystem.teleop_periodic();
        liftSubsystem.teleop_periodic();
        intakeSubsystem.teleop_periodic();
    }

    public static Attachments getInstance() {
        if (instance == null) {
            instance = new Attachments();
        }
        return instance;
    }

    private void startCamera(HardwareMap hardwareMap) {

        visionProcessor = new VisionProcessor();
        stackProcessor = new Stack_VisionProcessor();
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, names.webcam))
                .addProcessor(aprilTagProcessor)
                .addProcessor(visionProcessor)
//                .addProcessor(stackProcessor)
                .build();

    }

    // Getting Prop Location
    public void getPropLocation() {
        VisionProcessor.Selected selection = visionProcessor.getSelection();
        if (selection == VisionProcessor.Selected.LEFT) {
            Globals.PROP_LOCATION = Globals.PropLocation.LEFT;
        } else if (selection == VisionProcessor.Selected.MIDDLE) {
            Globals.PROP_LOCATION = Globals.PropLocation.CENTER;
        } else if (selection == VisionProcessor.Selected.RIGHT) {
            Globals.PROP_LOCATION = Globals.PropLocation.RIGHT;
        }
    }

    public int propLocationToInt(Globals.PropLocation pl) {
        switch (pl) {
            case LEFT:
                return 1;
            case CENTER:
                return 2;
            case RIGHT:
                return 3;
            default:
                return 0;
        }

    }
}