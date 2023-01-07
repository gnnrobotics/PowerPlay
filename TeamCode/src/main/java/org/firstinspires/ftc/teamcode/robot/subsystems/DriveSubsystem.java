package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {

    private final DcMotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;


    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    public DriveSubsystem(HardwareMap hMap) {
        m_frontLeft = hMap.get(DcMotorEx.class, "frontLeft");
        m_frontRight = hMap.get(DcMotorEx.class, "frontRight");
        m_backLeft = hMap.get(DcMotorEx.class, "backLeft");
        m_backRight = hMap.get(DcMotorEx.class, "backRight");

        m_frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        m_backRight.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void drive(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        m_frontLeft.setPower(frontLeftPower);
        m_frontRight.setPower(frontRightPower);
        m_backLeft.setPower(backLeftPower);
        m_backRight.setPower(backRightPower);
    }
}