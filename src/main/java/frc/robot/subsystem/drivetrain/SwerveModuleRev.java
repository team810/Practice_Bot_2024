package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

class SwerveModuleRev implements SwerveModuleIO {
	private final CANSparkMax drive;
	private final CANSparkMax steer;

	private final RelativeEncoder drive_encoder;
	private final CANcoder canCoder;
	private final SwerveModuleDetails details;

	private double driveVoltage;
	private double steerVoltage;

	public SwerveModuleRev(SwerveModuleDetails details)
	{

		drive = new CANSparkMax(details.driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steer = new CANSparkMax(details.steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		drive.restoreFactoryDefaults();
		steer.restoreFactoryDefaults();

		drive.clearFaults();
		steer.clearFaults();
		
		canCoder = new CANcoder(details.encoderID);
		this.details = details;

		drive.setSmartCurrentLimit(40);
		steer.setSmartCurrentLimit(30);

		drive.enableVoltageCompensation(12.0);
		steer.enableVoltageCompensation(12.0);


		drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
		steer.setIdleMode(CANSparkMax.IdleMode.kBrake);

		drive_encoder = drive.getEncoder();

		CANcoderConfiguration configuration = new CANcoderConfiguration();
		configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

		driveVoltage = 0;
		steerVoltage = 0;
	}
	@Override
	public void update() {
		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/WheelVelocity", getWheelVelocity());
		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/DriveVoltage", driveVoltage);
		Logger.getInstance().recordOutput("Drivetrain/"+ details.module.name() + "/DriveAmpDraw", drive.getOutputCurrent());
		Logger.getInstance().recordOutput("Drivetrain/"+ details.module.name() + "/DriveTemperature", drive.getMotorTemperature());

		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/SteerVoltage", steerVoltage);
		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/WheelAngle", getWheelAngle().getRadians());
		Logger.getInstance().recordOutput("Drivetrain/"+ details.module.name() + "/SteerAmpDraw", drive.getOutputCurrent());
		Logger.getInstance().recordOutput("Drivetrain/"+ details.module.name() + "/SteerTemperature", steer.getMotorTemperature());
	}

    @Override
    public void resetPosition() {
		drive_encoder.setPosition(0);
    }

    @Override
    public void setIdleMode(CANSparkMax.IdleMode mode) {
		drive.setIdleMode(mode);
    }

    @Override
	public void setDriveVoltage(double voltage) {
		driveVoltage = MathUtil.clamp(voltage, -1, 1);

		drive.set(driveVoltage);
	}

	@Override
	public void setSteerVoltage(double voltage) {
		steerVoltage = MathUtil.clamp(voltage, -1, 1);
		steerVoltage = -1 * steerVoltage;
		steer.set(steerVoltage);
	}

	@Override
	public Rotation2d getWheelAngle() {

		return new Rotation2d(
				MathUtil.angleModulus(canCoder.getPosition().getValue() * 2 * Math.PI)
		);
	}

	@Override
	public double getWheelVelocity() {
		return drive_encoder.getVelocity();
	}

	@Override
	public double getWheelPosition() {
		return drive_encoder.getPosition();
	}
}
