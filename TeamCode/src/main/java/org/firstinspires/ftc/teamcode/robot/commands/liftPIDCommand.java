package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class liftPIDCommand extends CommandBase {

    private final LiftSubsystem m_LiftSubsystem;

    // private final double ticks_in_degree = 537.7/360;

    private DoubleSupplier p, i, d, mg;
    private DoubleSupplier target;
    private PIDController controller;


    public liftPIDCommand(LiftSubsystem subsystem, DoubleSupplier targetPosition) {
        m_LiftSubsystem = subsystem;

        target = targetPosition;

        p = () -> 0.05;
        i = () -> 0;
        d = () -> 0.00045;
        mg = () -> 0.001;

        addRequirements(m_LiftSubsystem);
    }
    public liftPIDCommand(LiftSubsystem subsystem, DoubleSupplier targetPosition, DoubleSupplier pInput, DoubleSupplier iInput, DoubleSupplier dInput, DoubleSupplier mgInput) {
        m_LiftSubsystem = subsystem;

        target = targetPosition;

        p = pInput;
        i = iInput;
        d = dInput;
        mg = mgInput;

        addRequirements(m_LiftSubsystem);
    }

    @Override
    public void initialize() {
        controller = new PIDController(p.getAsDouble(), i.getAsDouble(), d.getAsDouble());
    }

    public void execute()
    {
        controller.setPID(p.getAsDouble(), i.getAsDouble(), d.getAsDouble());
        double liftPosition = m_LiftSubsystem.getEncoder();
        double currentTarget = target.getAsDouble();
        double pid = controller.calculate(liftPosition, currentTarget);

        double power = pid + mg.getAsDouble();

        m_LiftSubsystem.setSpecificHeight(power);
    }
}