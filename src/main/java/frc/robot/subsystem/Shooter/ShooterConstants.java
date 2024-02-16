package frc.robot.subsystem.Shooter;

import com.pathplanner.lib.util.PIDConstants;

public class ShooterConstants {

    public static final int TOP_MOTOR_ID = 12;
    public static final int BOTTOM_MOTOR_ID = 11;

    public static final PIDConstants TOP_CONTROLLER = new PIDConstants(.0003,.001,0);
    public static final PIDConstants BOTTOM_CONTROLLER = new PIDConstants(.0003,.001,0);

    public static final double SHOOTER_MAX_RPM = 3000;

}
