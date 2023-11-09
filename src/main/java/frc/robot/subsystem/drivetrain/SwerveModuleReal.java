package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleReal implements SwerveModuleIO {

	private SwerveModuleState targetState; // this is where the module should be
	private SwerveModuleState currentState; // this is the state with real values from the encoders
	private SwerveModulePosition modulePosition;
	private SpeedMode speedMode;

	private final CANSparkMax drive;
	private final CANSparkMax steer;

	private final RelativeEncoder driveEncoder;
	private final CANCoder steerEncoder;

	private final PIDController steerController;

	public SwerveModuleReal(ModuleDetails details)
	{
		speedMode = SpeedMode.normal;

		drive = new CANSparkMax(details.driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steer = new CANSparkMax(details.steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		driveEncoder = drive.getEncoder();
		steerEncoder = new CANCoder(details.encoderID);

		steerController = new PIDController(0,0,0);

		driveEncoder.setVelocityConversionFactor(
				(Math.PI * 4) * DrivetrainConstants.GEAR_REDUCTION_DRIVE * 60
		);
		driveEncoder.setPositionConversionFactor(
				(Math.PI * 4) * DrivetrainConstants.GEAR_REDUCTION_DRIVE
		);

		targetState = new SwerveModuleState();
		modulePosition = new SwerveModulePosition();

	}

	@Override
	public void periodic() {

	}

	@Override
	public SwerveModuleState getState() {
		return currentState;
	}
	@Override
	public void resetModulePositions() {
		modulePosition = new SwerveModulePosition();
	}

	@Override
	public void setSpeedMode(SpeedMode mode) {
		this.speedMode = mode;
	}

	@Override
	public SwerveModulePosition getModulePosition() {
		return modulePosition;
	}

	@Override
	public void setState(SwerveModuleState state) {
		targetState = state;
	}
}
