package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.componentConstants;

import java.util.function.DoubleSupplier;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class SusanSubsystem extends SubsystemBase {


    private final DcMotorEx susanMotor;
    private DoubleSupplier target;

    private boolean switchState = false;
    private boolean initialStateRequest = false;
    private boolean isUsed = false;
    private final double pidTolerance = 10;

    public SusanSubsystem(final HardwareMap hMap, final String name) {
        susanMotor = hMap.get(DcMotorEx.class, "susanMotor");
        susanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        susanMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        susanMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        target = () -> 0;
    }

    public void spinSusan(double spinSpeed) {

        susanMotor.setPower(spinSpeed);
    }
    public double getEncoder() {

        return susanMotor.getCurrentPosition();
    }
    public double getPower() {
        return susanMotor.getPower();
    }
    public DoubleSupplier getTarget() {
        return target;
    }
    public void setTarget(DoubleSupplier newTarget) {
        target = newTarget;
    }

    public void switchState() {
        switchState = !switchState;
    }
    public boolean getSwitchState() {
        return switchState;
    }

    public void initialStateRequest() {
        initialStateRequest = !initialStateRequest;
    }
    public boolean getInitialStateRequest() {
        return initialStateRequest;
    }
    public double getPidTolerance() { return pidTolerance; }
    public void setUseState(boolean isBeingUsed) { isUsed = isBeingUsed; }
    public boolean getUseState() { return isUsed; }

    public componentConstants.susanLevel getLevel() {
        return componentConstants.susanCurrentLevel;
    }
    public void setLevel(componentConstants.Level newLevel) {
        componentConstants.currentLevel = newLevel;
    }
}