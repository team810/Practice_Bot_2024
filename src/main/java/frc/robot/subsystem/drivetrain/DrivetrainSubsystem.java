package frc.robot.subsystem.drivetrain;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.navx.Navx;
import frc.lib.navx.NavxReal;
import frc.lib.navx.NavxSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystem.drivetrain.DrivetrainConstants.*;


public class DrivetrainSubsystem extends SubsystemBase {
	private static final DrivetrainSubsystem INSTANCE = new DrivetrainSubsystem();
	public static DrivetrainSubsystem getInstance() {return INSTANCE;}

	private DrivetrainMode mode;

	private SwerveModulePosition frontLeftPosition;
	private SwerveModulePosition frontRightPosition;
	private SwerveModulePosition backLeftPosition;
	private SwerveModulePosition backRightPosition;

	private SwerveModuleState frontLeftState;
	private SwerveModuleState frontRightState;
	private SwerveModuleState backLeftState;
	private SwerveModuleState backRightState;

	private ChassisSpeeds currentSpeeds;

	private ChassisSpeeds targetTeleopSpeeds;
	private ChassisSpeeds targetAutoSpeeds;
	private ChassisSpeeds targetAutoAlineSpeeds;
	private ChassisSpeeds targetTeleopAutoAlineSpeeds;

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule backLeft;
	private final SwerveModule backRight;

	private final SwerveDriveKinematics kinematics;
	private final SwerveDriveOdometry odometry;

	private final Navx navx;

	private SpeedMode speedMode;

	private DrivetrainSubsystem() {


		SwerveModuleDetails frontLeftDetails = new SwerveModuleDetails(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET, SwerveModuleEnum.frontLeft);
		SwerveModuleDetails frontRightDetails = new SwerveModuleDetails(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET, SwerveModuleEnum.frontRight);
		SwerveModuleDetails backLeftDetails = new SwerveModuleDetails(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET, SwerveModuleEnum.backLeft);
		SwerveModuleDetails backRightDetails = new SwerveModuleDetails(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET, SwerveModuleEnum.backRight);

		frontLeft = new SwerveModule(frontLeftDetails);
		frontRight = new SwerveModule(frontRightDetails);
		backLeft = new SwerveModule(backLeftDetails);
		backRight = new SwerveModule(backRightDetails);

		if (Robot.isSimulation())
		{
			navx = new NavxSim();
		}else{
			navx = new NavxReal();
		}

		frontLeftPosition = frontLeft.getModulePosition();
		frontRightPosition = frontRight.getModulePosition();
		backLeftPosition = backLeft.getModulePosition();
		backRightPosition = backRight.getModulePosition();

		frontLeftState = new SwerveModuleState();
		frontRightState = new SwerveModuleState();
		backLeftState = new SwerveModuleState();
		backRightState = new SwerveModuleState();

		currentSpeeds = new ChassisSpeeds();

		targetAutoSpeeds = new ChassisSpeeds();
		targetTeleopSpeeds = new ChassisSpeeds();
		targetTeleopAutoAlineSpeeds = new ChassisSpeeds();
		targetAutoAlineSpeeds = new ChassisSpeeds();

		kinematics = KINEMATICS;
		odometry = new SwerveDriveOdometry(kinematics, navx.getRotation2d(), new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition});
		mode = DrivetrainMode.teleop;

		setSpeedMode(SpeedMode.normal);
	}

	@Override
	public void periodic() {


		ChassisSpeeds targetSpeed;

		switch (mode)
		{
			case teleop:
				targetSpeed = targetTeleopSpeeds;
				break;
			case teleop_autoAline:
				targetSpeed = targetTeleopAutoAlineSpeeds;
				break;
			case auto:
				targetSpeed = targetAutoSpeeds;
				break;
			case auto_autoAline:
				targetSpeed = targetAutoAlineSpeeds;
				break;
			case telop_auto_turn:
				targetSpeed = new ChassisSpeeds(targetTeleopSpeeds.vxMetersPerSecond, targetTeleopSpeeds.vyMetersPerSecond,0);
				break;
			default:
				throw new RuntimeException("Triggered a default state, IDK how you did this get help from Matthew, " +
						"This will be funny when I eventually adjacently make the error");
		}

		if (RobotState.isDisabled())
		{
			targetSpeed = new ChassisSpeeds(0,0,0);
		}

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeed, getRotation())
		);


		// This mess with the pid controllers, it makes the mid controllers go back and forth
		states[0] = SwerveModuleState.optimize(states[0], frontLeft.getState().angle);
		states[1] = SwerveModuleState.optimize(states[1], frontRight.getState().angle);
		states[2] = SwerveModuleState.optimize(states[2], backLeft.getState().angle);
		states[3] = SwerveModuleState.optimize(states[3], backRight.getState().angle);

		frontLeft.setState(states[0]);
		frontRight.setState(states[1]);
		backLeft.setState(states[2]);
		backRight.setState(states[3]);

		// Commented out for testing purposes
		frontLeftState = states[0];
		frontRightState = states[1];
		backLeftState = states[2];
		backRightState = states[3];

		currentSpeeds = targetSpeed;

		if (Robot.isSimulation())
		{
			// gyro update
			double gyroRate = currentSpeeds.omegaRadiansPerSecond;
			navx.setRate(gyroRate);

			navx.update(Robot.defaultPeriodSecs);
		}

		frontLeft.periodic();
		frontRight.periodic();
		backLeft.periodic();
		backRight.periodic();

		frontLeftPosition = frontLeft.getModulePosition();
		frontRightPosition = frontRight.getModulePosition();
		backLeftPosition = backLeft.getModulePosition();
		backRightPosition = backRight.getModulePosition();

		odometry.update(getRotation(), new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition});

		Logger.recordOutput("Drivetrain/currentStates", frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
		Logger.recordOutput("Drivetrain/states", frontLeftState, frontRightState, backLeftState, backRightState);
		Logger.recordOutput("Drivetrain/gyro" ,getRotation().getRadians());
		Logger.recordOutput("RobotPose", getPose());

	}


	public void resetOdometry(Pose2d newPose)
	{

		frontLeft.resetModulePositions();
		frontRight.resetModulePositions();
		backLeft.resetModulePositions();
		backRight.resetModulePositions();

		frontLeftPosition = frontLeft.getModulePosition();
		frontRightPosition = frontRight.getModulePosition();
		backLeftPosition = backLeft.getModulePosition();
		backRightPosition = backRight.getModulePosition();

		odometry.resetPosition(getRotation(),new SwerveModulePosition[] {frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition}, newPose);
	}
	public DrivetrainMode getMode() {
		return mode;
	}

	public Rotation2d getRotation()
	{
		return navx.getRotation2d();
	}
	public void zeroGyro()
	{
		navx.zeroYaw();
	}
	public void setMode(DrivetrainMode mode) {
		this.mode = mode;
	}

	/**
	 * @param x this is the x input
	 * @param y this is the y input
	 * @param z rotate input
	 * This should get values that have already been altered and changed on a scale of -Max speed to Max speed
	 */
	public void setTargetTeleopSpeeds(double x, double y, double z)
	{
		targetTeleopSpeeds = new ChassisSpeeds(y,x,z);
	}

	public Pose2d getPose()
	{
		return odometry.getPoseMeters();
	}
	public void setTargetAutoSpeeds(double x, double y, double z)
	{
		targetAutoSpeeds = new ChassisSpeeds(x,y,z);
	}
	public void setTargetAutoSpeeds(ChassisSpeeds speeds)
	{
		targetAutoSpeeds = speeds;
	}
	public void setTargetAutoSpeeds(SwerveModuleState[] states)
	{
		targetAutoSpeeds = kinematics.toChassisSpeeds(states);
	}
	public ChassisSpeeds getTargetTeleopSpeeds() {
		return targetTeleopSpeeds;
	}
	public void setTargetTeleopSpeeds(ChassisSpeeds mTargetTeleopSpeeds) {
		targetTeleopSpeeds = mTargetTeleopSpeeds;
	}
	public ChassisSpeeds getTargetAutoSpeeds() {
		return targetAutoSpeeds;
	}

	public void setSpeedMode(SpeedMode speedMode) {
		this.speedMode = speedMode;
		frontLeft.setSpeedMode(this.speedMode);
		frontRight.setSpeedMode(this.speedMode);
		backLeft.setSpeedMode(this.speedMode);
		backRight.setSpeedMode(this.speedMode);
	}
	public SpeedMode getSpeedMode() {
		return speedMode;
	}
}
