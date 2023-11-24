package frc.robot.subsystem.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.MoreMath;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleSim implements SwerveModuleIO {
	private final FlywheelSim drive;
	private final FlywheelSim steer;

	private double driveVoltage;
	private double steerVoltage;

	private final SwerveModuleDetails details;

	/**This is the speed of the wheel after the gear ratio in rpm*/
	private double wheelVelocity;
	/**Drive position is the distance that the wheel has traveled*/
	private double drivePosition;
	/**This is the angle that the wheel is currently at*/
	private Rotation2d steerPosition;

	public SwerveModuleSim(SwerveModuleDetails details) {
		this.details = details;

		drive = new FlywheelSim(DCMotor.getNEO(1), 1, 0.001);
		steer = new FlywheelSim(DCMotor.getNEO(1), 150/7.0, 0.004);

		driveVoltage = 0;
		steerVoltage = 0;

		drivePosition = 0;
		wheelVelocity = 0;
		steerPosition = new Rotation2d(0);
	}

	@Override
	public void update() {
		drive.setInputVoltage(driveVoltage);
		steer.setInputVoltage(steerVoltage);

		drive.update(Robot.defaultPeriodSecs);
		steer.update(Robot.defaultPeriodSecs);

		wheelVelocity = drive.getAngularVelocityRPM();
		double currentSpeed =
				(((getWheelVelocity() / DrivetrainConstants.GEAR_REDUCTION_DRIVE) / 60 ) * // This is the wheel gear ratio concision factor
						((Math.PI* 4) / 12) * Robot.defaultPeriodSecs )
		;

		drivePosition = MoreMath.toMeters(currentSpeed) + drivePosition;

		double steerVelocity = steer.getAngularVelocityRadPerSec(); // This is the steer velocity of the wheel after the gear ratio is applied
		steerVelocity = Rotation2d.fromRotations(steerVelocity * Robot.defaultPeriodSecs).getRadians(); // Converting from rotations per second to radians per second
		steerPosition = Rotation2d.fromRadians(MathUtil.angleModulus(steerPosition.getRadians() + steerVelocity));

		wheelVelocity = drive.getAngularVelocityRPM();

		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/WheelVelocity", wheelVelocity);
		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/DriveVoltage", driveVoltage);
		Logger.getInstance().recordOutput("Drivetrain/"+ details.module.name() + "/DriveAmpDraw", drive.getCurrentDrawAmps());

		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/SteerVoltage", steerVoltage);
		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/WheelAngle", getWheelAngle().getRadians());
		Logger.getInstance().recordOutput("Drivetrain/"+ details.module.name() + "/SteerAmpDraw", steer.getCurrentDrawAmps());
	}

	@Override
	public void setDriveVoltage(double voltage) {
		driveVoltage = MathUtil.clamp(voltage, -12, 12);
		driveVoltage = MathUtil.applyDeadband((driveVoltage / 12), .001);
		driveVoltage = driveVoltage * 12;
	}
	@Override
	public void setSteerVoltage(double voltage) {
		steerVoltage = MathUtil.clamp(voltage, -12, 12);
		steerVoltage = MathUtil.applyDeadband((steerVoltage / 12), .001);
		steerVoltage = steerVoltage * 12;
	}

	@Override
	public Rotation2d getWheelAngle() {
		return steerPosition;
	}

	@Override
	public double getWheelPosition() {
		return drivePosition;
	}

	@Override
	public double getWheelVelocity() {
		return wheelVelocity;
	}
}