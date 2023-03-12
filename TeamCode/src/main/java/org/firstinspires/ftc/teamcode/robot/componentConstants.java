package org.firstinspires.ftc.teamcode.robot;

import java.util.function.DoubleSupplier;

public class componentConstants { // use 3123's code

    // Drive constant

    public static final double normalMotor = 1;
    public static final double fineMotor = 0.1;

    // Servo constants
    public static final double leftClosed = 0.29;
    public static final double leftOpen = 0.05;
    public static final double rightClosed = 0.61;
    public static final double rightOpen = 0.85;

    // Lift constants
    public static final double liftSpeed = 1;
    public static final DoubleSupplier downLevel = () -> 0;
    public static final DoubleSupplier groundJLevel = () -> 150;
    public static final DoubleSupplier lowLevel = () -> 1650;
    public static final DoubleSupplier mediumLevel = () -> 2850;
    public static final DoubleSupplier highLevel = () -> 3950;
    public static DoubleSupplier currentTarget = downLevel;

    public static double liftTime = 1;

    public static double endPosition;

    public enum Level {
        DOWN,
        GROUND_J,
        LOW,
        MEDIUM,
        HIGH
    }

    public static Level currentLevel = Level.DOWN;

    public static DoubleSupplier encoderValue;
    public static double pidTolerance = 5;

    // Susan constants
    public static final double susanSpeed = 0.35;

    public enum susanLevel {
        FRONT,
        BACK,
        SIDE
    }

    public static susanLevel susanCurrentLevel = susanLevel.BACK;

    public static final DoubleSupplier frontLevel = () -> 0;
    public static final DoubleSupplier sideLevel = () -> -135;
    public static final DoubleSupplier backLevel = () -> -270;

    // See respective lift and susan PIDs for their constants—putting them here would be too messy


    }

