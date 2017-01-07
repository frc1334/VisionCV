package org.usfirst.frc.team1334.Robot.Subsystems;


import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Created by FRC1334 on 8/16/2016.
 */
public class PickupSubsystem extends Subsystem {
    public static CANTalon Angle = new CANTalon(9);
    public static CANTalon Rollers = new CANTalon(10);


    protected void initDefaultCommand() {

    }
    public static void angleMove(double speed)
    {
        Angle.set(speed);
    }
    public static void roll (boolean isForward, boolean isBackward){
        if(isForward){
            Rollers.set(1.0);
        }else if(isBackward){
            Rollers.set(-1.0);
        }else{
            Rollers.set(0);
        }
    }
}
