package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.Level.DOWN;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.downLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.endPosition;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.groundJLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.highLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.liftSpeed;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.liftTime;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.lowLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.mediumLevel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.componentConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class liftStateMachine extends CommandBase {

    private final LiftSubsystem m_LiftSubsystem;
    private DoubleSupplier p, i, d, mg, maxVelocity, maxAcceleration;
    private DoubleSupplier target;
    private ProfiledPIDController controller;
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;
    private final double pidTolerance;
    private ElapsedTime liftTimer = new ElapsedTime();

    private enum LiftState {
        LIFT_START,
        LIFT_PID,
        LIFT_POSITION,
        LIFT_RETRACT
    }

    public static LiftState liftState = LiftState.LIFT_START;

    public liftStateMachine(LiftSubsystem subsystem, DoubleSupplier left, DoubleSupplier right) {
        m_LiftSubsystem = subsystem;
        m_left = left;
        m_right = right;

        p = () -> 0.05;
        i = () -> 0;
        d = () -> 0.00013;
        mg = () -> 0;
        maxVelocity = () -> 36000;
        maxAcceleration = () -> 14500;

        pidTolerance = m_LiftSubsystem.getPidTolerance();

        addRequirements(m_LiftSubsystem);
    }

    @Override
    public void initialize() {
        m_LiftSubsystem.setSpecificHeight(0);
        controller = new ProfiledPIDController(p.getAsDouble(), i.getAsDouble(), d.getAsDouble(), new TrapezoidProfile.Constraints(maxVelocity.getAsDouble(), maxAcceleration.getAsDouble()));
    }

    public void execute()
    {
        switch(m_LiftSubsystem.getLevel()) {
            case DOWN:
                target = downLevel;
                break;
            case GROUND_J:
                target = groundJLevel;
                break;
            case LOW:
                target = lowLevel;
                break;
            case MEDIUM:
                target = mediumLevel;
                break;
            case HIGH:
                target = highLevel;
                break;
        }

        switch (liftState) {
            case LIFT_START:
                if (m_left.getAsDouble() == 1 && m_right.getAsDouble() == 1) {
                    m_LiftSubsystem.setSpecificHeight(0);
                } else if (m_left.getAsDouble() == 1) {
                    m_LiftSubsystem.setSpecificHeight(liftSpeed); // change to constant
                } else if (m_right.getAsDouble() == 1) {
                    m_LiftSubsystem.setSpecificHeight(-liftSpeed); // change to negative of constant
                } else {
                    m_LiftSubsystem.setSpecificHeight(0);
                }

                if (m_LiftSubsystem.getSwitchState()) {
                    m_LiftSubsystem.switchState();
                    liftState = LiftState.LIFT_PID;
                }
                break;
            case LIFT_PID:
                pid();

                if (m_LiftSubsystem.getEncoder() < target.getAsDouble() + pidTolerance && m_LiftSubsystem.getEncoder() > target.getAsDouble() - pidTolerance) {
                    liftState = LiftState.LIFT_POSITION;
                }
                break;
            case LIFT_POSITION:
                if (m_left.getAsDouble() == 1 && m_right.getAsDouble() == 1) {
                    m_LiftSubsystem.setSpecificHeight(0);
                } else if (m_left.getAsDouble() == 1) {
                    m_LiftSubsystem.setSpecificHeight(liftSpeed); // change to constant
                } else if (m_right.getAsDouble() == 1) {
                    m_LiftSubsystem.setSpecificHeight(-liftSpeed); // change to negative of constant
                } else {
                    m_LiftSubsystem.setSpecificHeight(0);
                }

                if (m_LiftSubsystem.getSwitchState()) {
                    m_LiftSubsystem.switchState();
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                pid();

                m_LiftSubsystem.setLevel(DOWN);

                if (m_LiftSubsystem.getEncoder() < target.getAsDouble() + pidTolerance && m_LiftSubsystem.getEncoder() > target.getAsDouble() - pidTolerance) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_START;
        }

        if ((m_left.getAsDouble() == 1 || m_right.getAsDouble() == 1) && liftState != LiftState.LIFT_START) {
            m_LiftSubsystem.initialStateRequest();
            liftState = LiftState.LIFT_START;
        }
        if(liftState == LiftState.LIFT_PID && (m_left.getAsDouble() == 1 || m_right.getAsDouble() == 1)) {
            liftState = LiftState.LIFT_POSITION;
        }

    }

    public void end() {
        endPosition = target.getAsDouble();
    }
    public void pid() {
        double liftPosition = m_LiftSubsystem.getEncoder();
        controller.setPID(p.getAsDouble(), i.getAsDouble(), d.getAsDouble());
        controller.setTolerance(pidTolerance);
        double currentTarget = target.getAsDouble();
        double pid = controller.calculate(liftPosition, currentTarget);
        double power = pid + mg.getAsDouble();

        m_LiftSubsystem.setSpecificHeight(power);
    }
}