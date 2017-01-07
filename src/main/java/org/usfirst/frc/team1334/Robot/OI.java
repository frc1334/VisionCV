package org.usfirst.frc.team1334.Robot;

import org.usfirst.frc.team1334.Robot.Util.Xbox360Controller;
/**
 * Created by FRC1334 on 8/15/2016.
 */
public class OI {
    private static final Xbox360Controller Driver = new Xbox360Controller(0);
    private static final Xbox360Controller Operator = new Xbox360Controller(1);

    public static double getSpeed() {return Driver.getRightTrigger() - Driver.getLeftTrigger();}
    public static double getTurn() {return Driver.getLeftXAxis();}
    public static boolean isRollForward(){return Operator.getButtonA();}
    public static boolean isRollBackward(){return Operator.getButtonB();}
    public static boolean isShootHS(){return Driver.getButtonX();}
    public static boolean isShootLS(){return Driver.getButtonY();}
    public static double anglespeed(){return Operator.getRightYAxis();}
}
