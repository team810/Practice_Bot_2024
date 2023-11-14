package frc.robot.subsystem.drivetrain;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class DrivetrainConstants {

	public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.635; // Length of bot in meters
	public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635; // Width of bot in meters

	public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6;
	public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
	public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 15;
	public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(269.30);

	public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
	public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
	public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
	public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238.62);

	public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
	public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1;
	public static final int BACK_LEFT_MODULE_STEER_ENCODER = 16;
	public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.22);

	public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
	public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3;
	public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 14;
	public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(79.63);

	public static final double NORMAL_SPEED = 3.6;
	public static final double SLOW_SPEED = 2.0;

	public static final double GEAR_REDUCTION_DRIVE = 8.14;

	public static final PIDConstants STEER_CONTROLLER_SIM = new PIDConstants(0,0,0);
	public static final PIDConstants DRIVE_CONTROLLER_SIM = new PIDConstants(0,0,0);

	public static final PIDConstants STEER_CONTROLLER_REAL = new PIDConstants(0,0,0);
	public static final PIDConstants DRIVE_CONTROLLER_REAL = new PIDConstants(0,0,0);

	public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0,
					DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0,
					-DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0,
					DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0,
					-DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));


}
