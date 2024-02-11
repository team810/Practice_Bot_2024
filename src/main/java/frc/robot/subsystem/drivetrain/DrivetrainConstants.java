package frc.robot.subsystem.drivetrain;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class DrivetrainConstants {

	public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
	public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
	public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
	public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0;

	public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
	public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
	public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
	public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0;

	public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
	public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
	public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
	public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0;

	public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
	public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
	public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
	public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 0;

	/**
	 * Max Speed of the robot
	 * @Unites MPS
	 */
	public static final double NORMAL_SPEED = 2.0;
	/**
	 * Slow mode of the robot
	 * @Unites MPS
	 */
	public static final double SLOW_SPEED = 1.0;

	/**
	 * Units Meters
	 */
	public static final double WHEEL_DIAMETER  = .1016;

	/**
	 * The distance traveled for ever rotation of the wheel. PI * Wheel Diameter
	 */
	public static final double DISTANCE_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;


	/**
	 * This is the drive motor gear reduction for the module
	 */
	public static final double GEAR_REDUCTION_DRIVE = 6.75;

	public static final PIDConstants DRIVE_CONTROLLER_SIM = new PIDConstants(0.035,0,0);
	public static final PIDConstants STEER_CONTROLLER_SIM = new PIDConstants(4,1,0.1);

	public static final PIDConstants DRIVE_CONTROLLER_REAL = new PIDConstants(0.00009,.001,0);
	public static final PIDConstants STEER_CONTROLLER_REAL = new PIDConstants(.5,0,0);

	/**
	 * The measurement of the front Left wheel to the front right wheel or the back left wheel to the back right wheel
	 * @Unites Meters
	 */
	public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.635;
	/**
	 * This is the measurement from the Front Left wheel to the back left wheel or the front right wheel to the back right wheel
	 * @Unites Meters
	 */
	public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635; // Width of bot in meters

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
