package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
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

		
		canCoder = new CANcoder(details.encoderID);
		this.details = details;

		drive.setSmartCurrentLimit(40);
		steer.setSmartCurrentLimit(30);

		drive.enableVoltageCompensation(12.0);
		steer.enableVoltageCompensation(12.0);

		drive.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10);
		drive.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10);
		drive.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 400);

		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 200);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 200);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 400); // This is for alternate encoders witch we do not use
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 200);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 200);

		drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
		steer.setIdleMode(CANSparkMax.IdleMode.kBrake);

		drive_encoder = drive.getEncoder();

		CANcoderConfiguration configuration = new CANcoderConfiguration();

		// Encoder Configuration
		configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

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
		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/WheelAngle", getWheelAngle().getDegrees());
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
		driveVoltage = MathUtil.applyDeadband(MathUtil.clamp(voltage, -12, 12), 1);
		drive.setVoltage(driveVoltage);
	}

	@Override
	public void setSteerVoltage(double voltage) {
		steerVoltage = MathUtil.applyDeadband(MathUtil.clamp(voltage, -12, 12), 1);
		steer.setVoltage(steerVoltage);
	}

	@Override
	public Rotation2d getWheelAngle() {
		return new Rotation2d(canCoder.getAbsolutePosition().getValue());
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
