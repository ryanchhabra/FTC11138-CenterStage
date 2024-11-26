package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

public class AutonomousClawSubsystem extends SubsystemBase {

    private final Servo autonomousClaw;
    private AutoClawState autonomousclawstate = AutoClawState.HOLD;

    public enum AutoClawState {
        HOLD,
        RELEASE
    }

    public AutonomousClawSubsystem(HardwareMap hardwareMap, String autoClaw) {
        autonomousClaw = hardwareMap.get(Servo.class, autoClaw);
    }

    public void updateState(AutoClawState state) {
        this.autonomousclawstate = state;

        switch (autonomousclawstate) {
            case HOLD:
                autonomousClaw.setPosition(Constants.pixelHold);
                break;
            case RELEASE:
                autonomousClaw.setPosition(Constants.pixelDrop);
                break;
        }
    }

    public AutoClawState getState() {
        return autonomousclawstate;
    }

}
