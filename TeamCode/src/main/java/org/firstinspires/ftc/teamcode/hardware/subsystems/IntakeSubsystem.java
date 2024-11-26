package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Attachments;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

public class IntakeSubsystem extends SubsystemBase {

    private final Servo claw1Servo, claw2Servo;
    private final CRServoImplEx intake1Servo, intake2Servo;
    private final RevBlinkinLedDriver leds;

    public ClawState claw1state = ClawState.OPEN;
    public ClawState claw2state = ClawState.OPEN;

    public IntakeState intakestate = IntakeState.STOP;

    public enum ClawSide {
        FRONT,
        BACK,
        BOTH
    }

    public enum ClawState {
        OPEN,
        CLOSED,
        NONE
    }

    public enum IntakeState {
        IN,
        STOP,
        OUT
    }

    public IntakeSubsystem(HardwareMap hardwareMap, String c1, String c2, String in1, String in2, String led) {
        claw1Servo = hardwareMap.get(Servo.class, c1);
        claw2Servo = hardwareMap.get(Servo.class, c2);
        intake1Servo = hardwareMap.get(CRServoImplEx.class, in1);
        intake2Servo = hardwareMap.get(CRServoImplEx.class, in2);
        leds = hardwareMap.get(RevBlinkinLedDriver.class, led);
    }

    public void teleop_periodic() {
        if (Attachments.getInstance().droneSubsystem.getState() == DroneSubsystem.DroneState.FIRED) {
            setLEDs(Constants.greenPattern);
        } else if (claw1state == ClawState.OPEN) {
            setLEDs(Constants.whitePattern);
        } else {
            switch (Globals.ALLIANCE) {
                case BLUE:
                    setLEDs(Constants.bluePattern);
                    break;
                case RED:
                    setLEDs(Constants.redPattern);
                    break;
            }
        }
    }

    public void setLEDs (RevBlinkinLedDriver.BlinkinPattern pattern) {leds.setPattern(pattern);}

    public void updateIntakeState(IntakeState state) {
        this.intakestate = state;
        switch (state) {
            case IN:
                intake1Servo.setPower(-1);
                intake2Servo.setPower(1);
                break;
            case OUT:
                intake1Servo.setPower(1);
                intake2Servo.setPower(-1);
                break;
            case STOP:
                intake1Servo.setPower(0);
                intake2Servo.setPower(0);
                break;
        }
    }

    public void updateClawState(ClawState state, ClawSide side) {
        switch (side) {
            case FRONT:
                this.claw1state = state;
                claw1Servo.setPosition(getClawStatePosition(state));
                break;
            case BACK:
                this.claw2state = state;
                claw2Servo.setPosition(getClawStatePosition(state));
                break;
            case BOTH:
                claw1Servo.setPosition(getClawStatePosition(state));
                this.claw1state = state;
                claw2Servo.setPosition(getClawStatePosition(state));
                this.claw2state = state;
                break;
        }
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case OPEN:
                return Constants.clawOpen;
            case CLOSED:
                return Constants.clawClose;
            default:
                return 0;
        }
    }

    public ClawState getClawState(ClawSide side) {
        switch (side) {
            case BOTH:
                return (claw1state == (ClawState.OPEN) || (claw2state == ClawState.OPEN)) ? ClawState.OPEN : ClawState.CLOSED;
            case FRONT:
                return this.claw1state;
            case BACK:
                return this.claw2state;
            default:
                return ClawState.NONE;
        }
    }


}
