package robot.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class LiftSubsystem extends SubsystemBase {


    private final DcMotorEx liftMotor;
    private final DcMotorEx susanMotor;

    public LiftSubsystem(final HardwareMap hMap, final String name) {
        liftMotor = hMap.get(DcMotorEx.class, "liftMotor");
        susanMotor = hMap.get(DcMotorEx.class, "susanMotor");
    }
    // abstraction for setting lift height
    public void setHeight(double liftSpeed) // set up command where you can eventually just specify down, low, medium, and high and have it go there
    {
        liftMotor.setPower(liftSpeed);
    }

    public void spinSusan(double spinSpeed)
    {
        susanMotor.setPower(spinSpeed);
    }
}