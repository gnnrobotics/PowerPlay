package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class SusanSubsystem extends SubsystemBase {


    private final DcMotorEx susanMotor;

    public SusanSubsystem(final HardwareMap hMap, final String name) {
        susanMotor = hMap.get(DcMotorEx.class, "susanMotor");
    }

    public void spinSusan(double spinSpeed)
    {
        susanMotor.setPower(spinSpeed);
    }
}