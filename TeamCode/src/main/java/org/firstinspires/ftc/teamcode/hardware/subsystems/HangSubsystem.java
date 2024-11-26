package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;

public class HangSubsystem extends SubsystemBase {

    private final DcMotorEx hang1Motor, hang2Motor;

    private HangState hangstate;

    public enum HangState {
        HIGH,
        LOW,
        HANG
    }

    public HangSubsystem(HardwareMap hardwareMap, String h1, String h2) {
        hang1Motor = hardwareMap.get(DcMotorEx.class, h1);
        hang1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        hang2Motor = hardwareMap.get(DcMotorEx.class, h2);
        hangstate = HangState.LOW;
    }

    public void updateHangState(HangState state) {
        this.hangstate = state;
    }

    public void teleop_periodic() {
        switch (hangstate) {
            case HIGH:
                setPosition(1, Constants.hangHigh);
                break;
            case LOW:
                setPosition(1, Constants.hangMin);
                break;
            case HANG:
                setPosition(1, Constants.hangLow);
                break;
        }
    }

    public void setPosition(double power, int position) {
        hang1Motor.setPower(power);
        hang2Motor.setPower(power);
        hang1Motor.setTargetPosition(position);
        hang2Motor.setTargetPosition(position);
        hang1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPower(double power) {
        hang1Motor.setPower(power);
        hang2Motor.setPower(power);
    }

    public HangState getHangState() {
        return this.hangstate;
    }

    public int getHang1Position() {
        return hang1Motor.getCurrentPosition();
    }
    public int getHang2Position() {
        return hang2Motor.getCurrentPosition();
    }

}
