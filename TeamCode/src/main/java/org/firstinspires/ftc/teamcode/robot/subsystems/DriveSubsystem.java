package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.fineMotor;

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
        m_backRight = hMap.get(DcMotorEx.class,                     "backRight");

       m_frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
       m_backLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void drivePower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        m_frontLeft.setPower(frontLeftPower);
        m_frontRight.setPower(frontRightPower);
        m_backLeft.setPower(backLeftPower);
        m_backRight.setPower(backRightPower);
    }
    public double getFL(){
        return m_frontLeft.getPower();
    }
    public double getFR(){
        return m_frontRight.getPower();
    }
    public double getBL(){
        return m_backLeft.getPower();
    }
    public double getBR(){
        return m_backRight.getPower();
    }
}