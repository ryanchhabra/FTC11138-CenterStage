package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

@Config
public class LiftSubsystem extends SubsystemBase {

    private final DcMotorEx liftMotor;
    private int targetPosition = 0;
    private int currentPosition = 0;
    private double liftPower = 0.0;

    private boolean useLiftPower = true;
    private boolean liftModeUpdate = false;
    private boolean liftUseEnc = true;

    private int currentPixels = 2;
    private int currentHeight = 1000;


    private double maxPower = 1;


    public LiftSubsystem(HardwareMap hardwareMap, String lift) {
        liftMotor = hardwareMap.get(DcMotorEx.class, lift);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        this.liftPower = power;
    }

    public void setTargetPosition(int target) {
        this.targetPosition = target;
        useLiftPower = false;
        liftUseEnc = true;
    }

    public void setLiftPosition(double power, int target) {
        liftMotor.setPower(power);
        liftMotor.setTargetPosition(target);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void teleop_periodic() {
        currentPosition = liftMotor.getCurrentPosition();

        if (useLiftPower && liftPower == 0) {
            useLiftPower = false;
            targetPosition = currentPosition;
            liftUseEnc = true;
        }

        if (liftPower > 0.05) {
            liftUseEnc = true;
            // user trying to lift up
            if (currentPosition < Constants.liftMax || !Globals.LIMITS) {
                useLiftPower = true;
                liftPower *= Constants.liftUpRatio;
            } else {
                liftPower = 0;
            }
        } else if (liftPower < -0.05) {
            liftUseEnc = true;
            // user trying to lift down
            if (currentPosition > Constants.liftMin || !Globals.LIMITS) {
                useLiftPower = true;
                liftPower *= Constants.liftDownRatio;
                if (currentPosition > Constants.liftSlow) {
                    liftPower *= Constants.liftSlowRatio;
                }
            } else {
                liftPower = 0;
            }
        } else if (useLiftPower) {
            liftPower = 0;
        }


        if (liftModeUpdate && liftUseEnc) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftModeUpdate = false;
        }

        if (useLiftPower) {
            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor.setPower(this.liftPower);
        } else {
            setLiftPosition(this.maxPower, this.targetPosition);
            liftUseEnc = false;
            liftModeUpdate = true;
        }

    }


    public void changeCurrentHeight(int delta) {
//        this.currentPixels += delta;
//        this.currentHeight = Constants.backdropStart + Constants.pixelHeight * currentPixels;
        this.targetPosition += delta * Constants.pixelHeight;
    }

    public int getCurrentHeight() {
        return this.targetPosition;
    }

}
