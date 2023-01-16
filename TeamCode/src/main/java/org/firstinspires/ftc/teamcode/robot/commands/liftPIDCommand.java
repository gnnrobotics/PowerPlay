package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class liftPIDCommand extends CommandBase { // tell steph to use the double sided tape to do two strips and two layers of the foam on each claw

    private final LiftSubsystem m_LiftSubsystem;
    private final DoubleSupplier position;
    private double kP, kI, kD;
    private PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
    private PIDFController controller = new PIDFController(coeffs);



    public liftPIDCommand(LiftSubsystem subsystem, DoubleSupplier liftPosition) {
        m_LiftSubsystem = subsystem;
        position = liftPosition;

        kP = 0;
        kI = 0;
        kD = 0;

        addRequirements(m_LiftSubsystem);
    }

    @Override
    public void initialize() {
        controller.setTargetPosition(position.getAsDouble());
    }

    public void execute()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        ElapsedTime PIDTimer = new ElapsedTime();

    }

}