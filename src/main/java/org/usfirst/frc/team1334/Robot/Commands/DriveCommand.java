package org.usfirst.frc.team1334.Robot.Commands;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1334.Robot.OI;
import org.usfirst.frc.team1334.Robot.Robot;
import org.usfirst.frc.team1334.Robot.Util.Subsystems;
/**
 * Created by FRC1334 on 8/15/2016.
 */
public class DriveCommand extends Command {

    public DriveCommand(){
        requires(Robot.DRIVE_SUBSYSTEM);
        requires(Robot.PICKUP_SUBSYSTEM);
        requires(Robot.SHOOTER_SUBSYSTEM);
    }

    @Override
    protected void initialize() {
        /*
        Control Modes are as follows, this can be used to diagnose Talon malfunctions in the browser view of the RIO
        0 - PercentVbus
        2 - Speed
        15 - Disabled
         */
        Robot.DRIVE_SUBSYSTEM.left1.setControlMode(0);
        Robot.DRIVE_SUBSYSTEM.left2.setControlMode(0);
        Robot.DRIVE_SUBSYSTEM.right1.setControlMode(0);
        Robot.DRIVE_SUBSYSTEM.right2.setControlMode(0);
        Robot.SHOOTER_SUBSYSTEM.Flywheel.setControlMode(2);
        Robot.SHOOTER_SUBSYSTEM.Flywheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        Robot.SHOOTER_SUBSYSTEM.Flywheel.reverseSensor(true);
        Robot.SHOOTER_SUBSYSTEM.Speedwheel.setControlMode(2);
        Robot.SHOOTER_SUBSYSTEM.Speedwheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        //Robot.SHOOTER_SUBSYSTEM.Speedwheel.setInverted(false);
        Robot.PICKUP_SUBSYSTEM.Angle.setControlMode(0);
        Robot.PICKUP_SUBSYSTEM.Rollers.setControlMode(0);

    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_SUBSYSTEM.arcadeDrive(OI.getSpeed(),OI.getTurn());
        Robot.PICKUP_SUBSYSTEM.roll(OI.isRollForward(),OI.isRollBackward());
        Robot.PICKUP_SUBSYSTEM.angleMove(-(OI.anglespeed()) * 0.75);
        Robot.SHOOTER_SUBSYSTEM.Shoot(OI.isShootHS(),OI.isShootLS());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {

    }

    @Override
    protected void interrupted() {

    }
}
