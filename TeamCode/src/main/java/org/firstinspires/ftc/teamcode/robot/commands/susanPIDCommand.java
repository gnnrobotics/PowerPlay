package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

import java.util.function.DoubleSupplier;

public class susanPIDCommand extends CommandBase {

    private final SusanSubsystem susanSubsystem;

    private DoubleSupplier p, i, d;
    private DoubleSupplier target;
    private PIDController controller;


    public susanPIDCommand(SusanSubsystem subsystem, DoubleSupplier targetPosition) {
        susanSubsystem = subsystem;

        target = targetPosition;

        p = () -> 0.10;
        i = () -> 0;
        d = () -> 0.00005;

        addRequirements(susanSubsystem);
    }
    public susanPIDCommand(SusanSubsystem subsystem, DoubleSupplier targetPosition, DoubleSupplier pInput, DoubleSupplier iInput, DoubleSupplier dInput) {
        susanSubsystem = subsystem;

        target = targetPosition;

        p = pInput;
        i = iInput;
        d = dInput;

        addRequirements(susanSubsystem);
    }

    @Override
    public void initialize() {
        controller = new PIDController(p.getAsDouble(), i.getAsDouble(), d.getAsDouble());
    }

    public void execute()
    {
        controller.setPID(p.getAsDouble(), i.getAsDouble(), d.getAsDouble());
        double liftPosition = susanSubsystem.getEncoder();
        double currentTarget = target.getAsDouble();
        double pid = controller.calculate(liftPosition, currentTarget);

        susanSubsystem.spinSusan(pid);
    }
}