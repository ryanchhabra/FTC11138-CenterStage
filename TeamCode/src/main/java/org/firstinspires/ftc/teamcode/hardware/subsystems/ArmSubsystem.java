package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final Servo armServo, wristServo;
    private ArmState armstate;

    public enum ArmState {
        DROP,
        INTAKE,
        DRIVE
    }

    public ArmSubsystem(HardwareMap hardwareMap, String arm, String wrist) {
        armServo = hardwareMap.get(Servo.class, arm);
        wristServo = hardwareMap.get(Servo.class, wrist);
        this.armstate = ArmState.INTAKE;
    }

    public void updateArmState(ArmState state) {
        this.armstate = state;
    }
    public ArmState getArmState() {
        return this.armstate;
    }

    public void setArmServo(double position) {
        armServo.setPosition(position);
    }
    public void setWristServo(double position) {
        wristServo.setPosition(position);
    }

    public double getArmPosition() {
        return armServo.getPosition();
    }
    public double getWristPosition() {
        return wristServo.getPosition();
    }

}
