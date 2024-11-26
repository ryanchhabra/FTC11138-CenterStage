package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="VisionOpmode", group="test")
public class VisionOpMode extends OpMode {

    private VisionProcessor visionProcessor;
    private Stack_VisionProcessor stackProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        visionProcessor = new VisionProcessor();
        stackProcessor = new Stack_VisionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(visionProcessor)
                .addProcessor(stackProcessor)
                .build();

    }

    @Override
    public void init_loop() {
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("Detected ", visionProcessor.getSelection());
    }
}
