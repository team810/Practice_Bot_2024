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

	/**
	 * This is the speed of the wheel after the gear ratio in rpm
	 */
	private double wheelVelocity;
	/**
	 * Drive position is the distance that the wheel has traveled
	 */
	private double drivePosition;
	/**
	 * This is the angle that the wheel is currently at
	 */
	private Rotation2d steerPosition;

	public SwerveModuleSim(SwerveModuleDetails details) {
		this.details = details;

		drive = new FlywheelSim(DCMotor.getNEO(1), 1, 0.025);
		steer = new FlywheelSim(DCMotor.getNEO(1), 1, 0.004);

		driveVoltage = 0;
		steerVoltage = 0;

		drivePosition = 0;
		wheelVelocity = 0;
		steerPosition = new Rotation2d(0);

	}

	@Override
	public void update() {
		drive.update(Robot.defaultPeriodSecs);
		steer.update(Robot.defaultPeriodSecs);

		wheelVelocity = drive.getAngularVelocityRPM();
		drivePosition =
				(getWheelVelocity() / 6.75)* // This is the wheel gear ratio concision factor
						((Math.PI* 4) / 12) // This is the conference in feet
		;
		drivePosition = MoreMath.toMeters(drivePosition); // This converts the feet into meters

		double steerVelocity = steer.getAngularVelocityRPM() / (21.428571428571427); // This is the steer velocity of the wheel after the gear ratio is applied
		steerPosition = steerPosition.rotateBy(new Rotation2d(steerVelocity/ (2 * Math.PI))); // This should be correct ? hopefully

		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/WheelVelocity", getWheelVelocity());
		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/DriveVoltage", driveVoltage);
		Logger.getInstance().recordOutput("Drivetrain/"+ details.module.name() + "/DriveAmpDraw", drive.getCurrentDrawAmps());

		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/SteerVoltage", steerVoltage);
		Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/WheelAngle", getWheelAngle().getDegrees());
		Logger.getInstance().recordOutput("Drivetrain/"+ details.module.name() + "/SteerAmpDraw", steer.getCurrentDrawAmps());
	}

	@Override
	public void setDriveVoltage(double voltage) {
		driveVoltage = MathUtil.applyDeadband(MathUtil.clamp(voltage, -12, 12), 1);;
	}
	@Override
	public void setSteerVoltage(double voltage) {
		steerVoltage = MathUtil.applyDeadband(MathUtil.clamp(voltage, -12, 12), 1);
		drive.setInputVoltage(voltage);
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
		return drive.getAngularVelocityRPM();
	}

}