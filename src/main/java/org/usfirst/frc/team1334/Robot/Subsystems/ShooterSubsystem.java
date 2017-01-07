package org.usfirst.frc.team1334.Robot.Subsystems;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Created by FRC1334 on 8/16/2016.
 */
public class ShooterSubsystem extends Subsystem {
    public static CANTalon Speedwheel = new CANTalon(7);
    public static CANTalon Flywheel = new CANTalon(8);


    protected void initDefaultCommand() {

    }
    public static void Shoot(boolean isHS, boolean isLS)
    {
        if(isHS){
            Speedwheel.set(-4000);
            Flywheel.set(-4000);
        }else if(isLS){
            Speedwheel.set(-4000);
            Flywheel.set(-4000);
        }else{
            Speedwheel.set(0);
            Flywheel.set(0);
        }
    }
}
