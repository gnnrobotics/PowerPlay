package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.downLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.groundJLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.highLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.lowLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.mediumLevel;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.componentConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

import java.util.function.DoubleSupplier;

public class susanPIDCommand extends CommandBase {

    private final SusanSubsystem susanSubsystem;

    private DoubleSupplier p, i, d, maxVelocity, maxAcceleration;
    private DoubleSupplier target;
    private ProfiledPIDController controller;
    private componentConstants.Level nowLevel;


    public susanPIDCommand(SusanSubsystem subsystem, DoubleSupplier targetPosition) {
        susanSubsystem = subsystem;
// 135 ticks = 90 degrees
        target = targetPosition;

        p = () -> 0.08;
        i = () -> 0;
        d = () -> 0.0000001;
        maxVelocity = () -> 800;
        maxAcceleration = () -> 800;

        addRequirements(susanSubsystem);
    }
    public susanPIDCommand(SusanSubsystem subsystem) {
        susanSubsystem = subsystem;

        target = () -> 0;

        p = () -> 0.08;
        i = () -> 0;
        d = () -> 0.0000001;
        maxVelocity = () -> 800;
        maxAcceleration = () -> 800;

        addRequirements(susanSubsystem);
    }
    public susanPIDCommand(SusanSubsystem subsystem, DoubleSupplier targetPosition, DoubleSupplier pInput, DoubleSupplier iInput, DoubleSupplier dInput, DoubleSupplier maxVelInput, DoubleSupplier maxAccelInput) {
        susanSubsystem = subsystem;

        target = targetPosition;

        p = pInput;
        i = iInput;
        d = dInput;
        maxVelocity = maxVelInput;
        maxAcceleration = maxAccelInput;

        addRequirements(susanSubsystem);
    }

    @Override
    public void initialize() {
        controller = new ProfiledPIDController(p.getAsDouble(), i.getAsDouble(), d.getAsDouble(), new TrapezoidProfile.Constraints(maxVelocity.getAsDouble(), maxAcceleration.getAsDouble()));
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