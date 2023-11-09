package frc.robot.subsystem.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CanCoderSim;
import frc.lib.MoreMath;
import frc.robot.Robot;

public class SwerveModuleSim implements SwerveModuleIO {

	private SwerveModuleState targetState; // this is where the module should be
	private SwerveModuleState currentState; // this is the state with real values from the encoders

	private final CANSparkMax drive;

	private final CanCoderSim steer_encoder;
	private final RelativeEncoder drive_encoder;

	private final PIDController steer_controller;

	private SwerveModulePosition modulePosition;

	private SpeedMode speedMode;

	public SwerveModuleSim(ModuleDetails details)
	{
		// General Init
		targetState = new SwerveModuleState();
		currentState = new SwerveModuleState();
		modulePosition = new SwerveModulePosition();
		speedMode = SpeedMode.normal;

		// Drive motor Init
		drive = new CANSparkMax(details.driveID, CANSparkMaxLowLevel.MotorType.kBrushless);

		REVPhysicsSim.getInstance().addSparkMax(drive, DCMotor.getNEO(1));

		drive_encoder = drive.getEncoder();


		// Steer motor init
		steer_encoder = new CanCoderSim(details.encoderID, 0);
		steer_controller = new PIDController(20,0,0);
		steer_controller.setTolerance(.1);

		steer_encoder.setRate(0);

		steer_controller.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void periodic() {
		// set drive speed
		if (speedMode == SpeedMode.slow)
		{
			drive.setVoltage(targetState.speedMetersPerSecond / DrivetrainConstants.SLOW_SPEED);

		}
		if (speedMode == SpeedMode.normal)
		{
			drive.setVoltage(targetState.speedMetersPerSecond / DrivetrainConstants.NORMAL_SPEED);

		}
		// set steer speed
		steer_encoder.setRate(
						steer_controller.calculate(
								steer_encoder.getRot().getRadians(), targetState.angle.getRadians()
						));

		steer_encoder.update(Robot.defaultPeriodSecs);
		// update current state
		double velocity = drive_encoder.getVelocity();
		velocity = (velocity / DrivetrainConstants.GEAR_REDUCTION_DRIVE) / 60;
		velocity = velocity * Math.PI * 4;
		velocity = velocity / 12;
		velocity = MoreMath.toMeters(velocity);

		currentState = new SwerveModuleState(velocity, steer_encoder.getRot());
		// update module position
		modulePosition = new SwerveModulePosition(MoreMath.toMeters(((drive_encoder.getPosition() / DrivetrainConstants.GEAR_REDUCTION_DRIVE) * Math.PI * 4) / 12), steer_encoder.getRot());

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
