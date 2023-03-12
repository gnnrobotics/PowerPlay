package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.Level.DOWN;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.downLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.endPosition;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.groundJLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.highLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.liftSpeed;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.lowLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.mediumLevel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.componentConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class liftPIDCommand extends CommandBase {

    private final LiftSubsystem m_LiftSubsystem;

    // private final double ticks_in_degree = 537.7/360;

    private DoubleSupplier p, i, d, mg, maxVelocity, maxAcceleration;
    private ProfiledPIDController controller;
    private boolean customTarget = false;
    private double pidTolerance;

    public liftPIDCommand(LiftSubsystem subsystem, DoubleSupplier targetPosition) {
        m_LiftSubsystem = subsystem;
        m_LiftSubsystem.setTarget(targetPosition);

        p = () -> 0.05;
        i = () -> 0;
        d = () -> 0.00013;
        mg = () -> 0.00;
        maxVelocity = () -> 16250;
        maxAcceleration = () -> 10000;

        pidTolerance = m_LiftSubsystem.getPidTolerance();

        addRequirements(m_LiftSubsystem);
    }
    public liftPIDCommand(LiftSubsystem subsystem) {
        m_LiftSubsystem = subsystem;

        p = () -> 0.05;
        i = () -> 0;
        d = () -> 0.00013;
        mg = () -> 0.00;
        maxVelocity = () -> 16250;
        maxAcceleration = () -> 10000;

        pidTolerance = m_LiftSubsystem.getPidTolerance();

        addRequirements(m_LiftSubsystem);
    }
    public liftPIDCommand(LiftSubsystem subsystem, DoubleSupplier targetPosition, DoubleSupplier pInput, DoubleSupplier iInput, DoubleSupplier dInput, DoubleSupplier mgInput, DoubleSupplier maxVelInput, DoubleSupplier maxAccelInput) {
        m_LiftSubsystem = subsystem;
        m_LiftSubsystem.setTarget(targetPosition);

        p = pInput;
        i = iInput;
        d = dInput;
        mg = mgInput;
        maxVelocity = maxVelInput;
        maxAcceleration = maxAccelInput;

        pidTolerance = m_LiftSubsystem.getPidTolerance();

        addRequirements(m_LiftSubsystem);
    }

    @Override
    public void initialize() {
        controller = new ProfiledPIDController(p.getAsDouble(), i.getAsDouble(), d.getAsDouble(), new TrapezoidProfile.Constraints(maxVelocity.getAsDouble(), maxAcceleration.getAsDouble()));
    }

    public void execute()
    {
        double target = m_LiftSubsystem.getTarget().getAsDouble();

        controller.setPID(p.getAsDouble(), i.getAsDouble(), d.getAsDouble());
        double liftPosition = m_LiftSubsystem.getEncoder();
        double pid = controller.calculate(liftPosition, target);

        double power = pid + mg.getAsDouble();

        m_LiftSubsystem.setSpecificHeight(power);

    }

    public void end() {
        endPosition = m_LiftSubsystem.getTarget().getAsDouble();
    }
}