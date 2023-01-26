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

public class liftPIDCommand extends CommandBase { // tell steph to use the double sided tape to do two strips and two layers of the foam on each claw

    private final LiftSubsystem m_LiftSubsystem;

    private final double ticks_in_degree = 780/180.0;
    private PIDController controller;
    private double state;
    private double p, i, d, f;
    private int target;
    private ElapsedTime PIDTimer = new ElapsedTime();


    public liftPIDCommand(LiftSubsystem subsystem, DoubleSupplier targetPosition, DoubleSupplier pInput, DoubleSupplier iInput, DoubleSupplier dInput, DoubleSupplier fInput) {
        m_LiftSubsystem = subsystem;
        p = pInput.getAsDouble();
        i = iInput.getAsDouble();
        d = dInput.getAsDouble();
        f = fInput.getAsDouble();

        /* referencePosition = referencePositionInput.getAsDouble();
        referenceVelocity = referenceVelocityInput.getAsDouble();
        state = 0;
        lastError = 0;
        kP = 0;
        kI = 0;
        kD = 0; */

        addRequirements(m_LiftSubsystem);
    }

    @Override
    public void initialize() {
        controller = new PIDController(p, i ,d);

    }

    public void execute()
    {
        controller.setPID(p, i, d);
        double pid = controller.calculate(m_LiftSubsystem.getEncoder(), target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid * ff;

        m_LiftSubsystem.setSpecificHeight(power);

        /* FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);

        state = m_LiftSubsystem.getEncoder();
        m_LiftSubsystem.setSpecificHeight(calculatePID()); */
    }

   /* public double calculatePID()
    {

        double error = referencePosition - state;
        integralSum += error * PIDTimer.seconds();
        double derivative = (error - lastError) / PIDTimer.seconds();
        lastError = error;
        return kP*error + kI*integralSum;
    }
    public double calculateFeedforward()
    {
        return referenceVelocity*kV;
    }*/

}