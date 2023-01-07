package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystemCopy1 extends SubsystemBase {

    private final MecanumDrive m_drive;

    private final MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;


    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    public DriveSubsystemCopy1(HardwareMap hMap) {
        m_frontLeft = hMap.get(MotorEx.class, "frontLeft");
        m_frontRight = hMap.get(MotorEx.class, "frontRight");
        m_backLeft = hMap.get(MotorEx.class, "backLeft");
        m_backRight = hMap.get(MotorEx.class, "backRight");

        m_drive = new MecanumDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        m_drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

}