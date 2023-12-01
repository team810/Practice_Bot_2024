package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleRev implements SwerveModuleIO {
	private final CANSparkMax drive;
	private final CANSparkMax steer;

	private final RelativeEncoder drive_encoder;
	private final CANCoder canCoder;
	private final SwerveModuleDetails details;

	private double driveVoltage;
	private double steerVoltage;

	public SwerveModuleRev(SwerveModuleDetails details)
	{

		drive = new CANSparkMax(details.driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steer = new CANSparkMax(details.steerID, CANSparkMaxLowLevel.MotorType.kBrushless);
		
		canCoder = new CANCoder(details.encoderID);
		this.details = details;

		drive.setSmartCurrentLimit(40);
		steer.setSmartCurrentLimit(20);

		drive.enableVoltageCompensation(12.0);
		steer.enableVoltageCompensation(12.0);

		drive.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10);

		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 200);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 200);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 200);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 200);
		steer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 200);

		drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
		steer.setIdleMode(CANSparkMax.IdleMode.kBrake);

		drive_encoder = drive.getEncoder();

		CANCoderConfiguration configuration = new CANCoderConfiguration();

		// Encoder Configuration
		configuration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
		configuration.magnetOffsetDegrees = details.encoderOffset;

		canCoder.configAllSettings(configuration);

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
		return new Rotation2d(canCoder.getAbsolutePosition());
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
