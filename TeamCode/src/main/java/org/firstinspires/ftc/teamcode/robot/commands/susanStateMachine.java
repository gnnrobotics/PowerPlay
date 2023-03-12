package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.Level.DOWN;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.backLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.downLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.endPosition;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.frontLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.groundJLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.highLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.liftSpeed;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.lowLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.mediumLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.sideLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.susanSpeed;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.componentConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class susanStateMachine extends CommandBase {

    private final SusanSubsystem m_SusanSubsystem;
    private DoubleSupplier p, i, d, mg, maxVelocity, maxAcceleration;
    private DoubleSupplier target;
    private ProfiledPIDController controller;
    private final BooleanSupplier m_left;
    private final BooleanSupplier m_right;
    private final double pidTolerance;
    private ElapsedTime liftTimer = new ElapsedTime();

    private enum SusanState {
        MANUAL,
        PID
    }

    public static SusanState susanState = SusanState.MANUAL;

    public susanStateMachine(SusanSubsystem subsystem, BooleanSupplier left, BooleanSupplier right) {
        m_SusanSubsystem = subsystem;
        m_left = left;
        m_right = right;

        p = () -> 0.08;
        i = () -> 0;
        d = () -> 0.0000001;
        maxVelocity = () -> 800;
        maxAcceleration = () -> 800;

        pidTolerance = m_SusanSubsystem.getPidTolerance();

        addRequirements(m_SusanSubsystem);
    }

    @Override
    public void initialize() {
        m_SusanSubsystem.spinSusan(0);
        controller = new ProfiledPIDController(p.getAsDouble(), i.getAsDouble(), d.getAsDouble(), new TrapezoidProfile.Constraints(maxVelocity.getAsDouble(), maxAcceleration.getAsDouble()));
    }

    public void execute()
    {
        switch(m_SusanSubsystem.getLevel()) {
            case FRONT:
                target = frontLevel;
                break;
            case SIDE:
                target = sideLevel;
                break;
            case BACK:
                target = backLevel;
                break;
        }

        switch (susanState) {
            case MANUAL:
                if (m_left.getAsBoolean() && m_right.getAsBoolean()) {
                    m_SusanSubsystem.spinSusan(0);
                }
                else if (m_left.getAsBoolean()) {
                    m_SusanSubsystem.spinSusan(susanSpeed); // change to constant
                }
                else if (m_right.getAsBoolean()) {
                    m_SusanSubsystem.spinSusan(-susanSpeed); // change to negative of constant
                }
                else {
                    m_SusanSubsystem.spinSusan(0);
                }

                if (m_SusanSubsystem.getSwitchState()) {
                    m_SusanSubsystem.switchState();
                    susanState = SusanState.PID;
                }
                break;
            case PID:
                pid();

                if (m_SusanSubsystem.getEncoder() < target.getAsDouble() + pidTolerance && m_SusanSubsystem.getEncoder() > target.getAsDouble() - pidTolerance) {
                    susanState = susanState.MANUAL;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                susanState = SusanState.MANUAL;
        }

        if (m_SusanSubsystem.getInitialStateRequest() && susanState != susanState.MANUAL) {
            m_SusanSubsystem.initialStateRequest();
            susanState = susanState.MANUAL;
        }
    }

    public void end() {
        endPosition = target.getAsDouble();
    }
    public void pid() {
        double liftPosition = m_SusanSubsystem.getEncoder();
        controller.setPID(p.getAsDouble(), i.getAsDouble(), d.getAsDouble());
        controller.setTolerance(pidTolerance);
        double currentTarget = target.getAsDouble();
        double pid = controller.calculate(liftPosition, currentTarget);
        double power = pid + mg.getAsDouble();

        m_SusanSubsystem.spinSusan(power);
    }
}