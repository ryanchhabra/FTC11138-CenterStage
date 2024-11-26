package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

public class DroneSubsystem extends SubsystemBase {

    private final Servo droneServo;
    private DroneState dronestate;

    public enum DroneState {
        ARMED,
        FIRED
    }

    public DroneSubsystem(HardwareMap hardwareMap, String drone) {
        droneServo = hardwareMap.get(Servo.class, drone);
        updateState(DroneState.ARMED);
    }

    public DroneState getState() {
        return dronestate;
    }

    public void updateState(DroneState state) {
        this.dronestate = state;

        switch (dronestate) {
            case ARMED:
                droneServo.setPosition(Constants.planeHold);
                break;
            case FIRED:
                droneServo.setPosition(Constants.planeRelease);
                break;
        }
    }

}
