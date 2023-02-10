package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
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
public class LiftSubsystem extends SubsystemBase {


    private final DcMotorEx liftMotor;


    public LiftSubsystem(final HardwareMap hMap, final String name) {
        liftMotor = hMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    // abstraction for setting lift height
    public void setSpecificHeight(double liftSpeed) // set up command where you can eventually just specify down, low, medium, and high and have it go there
    {
        liftMotor.setPower(liftSpeed);
    }
    public double getEncoder() {

        return liftMotor.getCurrentPosition();
    }
    public double getPower() {
        return liftMotor.getPower();
    }
    public componentConstants.Level getLevel() {
        return componentConstants.currentLevel;
    }
    public void setLevel(componentConstants.Level newLevel) {
        componentConstants.currentLevel = newLevel;
    }
    public DoubleSupplier getTarget() {
        return componentConstants.currentTarget;
    }
    public void setTarget(DoubleSupplier newTarget) {
        componentConstants.currentTarget = newTarget;
    }
}