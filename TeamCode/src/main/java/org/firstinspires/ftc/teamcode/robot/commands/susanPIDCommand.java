package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.downLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.groundJLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.highLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.lowLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.mediumLevel;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.componentConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

import java.util.function.DoubleSupplier;

public class susanPIDCommand extends CommandBase {

    private final SusanSubsystem susanSubsystem;

    private DoubleSupplier p, i, d;
    private DoubleSupplier target;
    private PIDController controller;
    private componentConstants.Level nowLevel;


    public susanPIDCommand(SusanSubsystem subsystem, DoubleSupplier targetPosition) {
        susanSubsystem = subsystem;

        target = targetPosition;

        p = () -> 0.10;
        i = () -> 0;
        d = () -> 0.00005;

        addRequirements(susanSubsystem);
    }
    public susanPIDCommand(SusanSubsystem subsystem) {
        susanSubsystem = subsystem;

        nowLevel = componentConstants.currentLevel;

        if(nowLevel == componentConstants.Level.DOWN)
        {
            target = downLevel;
        }
        else if(nowLevel == componentConstants.Level.GROUND_J) {
            target = groundJLevel;
        }
        else if(nowLevel == componentConstants.Level.LOW) {
            target = lowLevel;
        }
        else if(nowLevel == componentConstants.Level.MEDIUM) {
            target = mediumLevel;
        }
        else if(nowLevel == componentConstants.Level.HIGH) {
            target = highLevel;
        }
        else {
            target = downLevel;
        }

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