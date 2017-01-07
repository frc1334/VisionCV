package org.usfirst.frc.team1334.Robot.Subsystems;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Created by FRC1334 on 8/15/2016.
 */
public class DriveSubsystem extends Subsystem{
    public static CANTalon left1 = new CANTalon(0);
    public static CANTalon left2 = new CANTalon(1);
    public static CANTalon right1 = new CANTalon(2);
    public static CANTalon right2 = new CANTalon(3);


    protected void initDefaultCommand() {
        left1.setSafetyEnabled(true);
        left2.setSafetyEnabled(true);
        right1.setSafetyEnabled(true);
        right2.setSafetyEnabled(true);
        left1.setExpiration(0.1);
        left2.setExpiration(0.1);
        right2.setExpiration(0.1);
        right1.setExpiration(0.1);
    }

    public static void tankDrive(double right, double left)
    {

        left1.set(-left);
        left2.set(-left);
        right1.set(right);
        right2.set(right);
    }

    public static void arcadeDrive(double speed, double turn)
    {
        tankDrive(speed - turn, speed + turn);
    }

}
