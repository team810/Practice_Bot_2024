package frc.robot.subsystem.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
	/** @param voltage -12 to 12*/
	void setDriveVoltage(double voltage);
	/** @param voltage -12 to 12*/
	void setSteerVoltage(double voltage);
	/**@return This is the current angle of the wheel -PI to PI*/
	Rotation2d getWheelAngle();
	/**@return The speed of the motor not the wheel in the units of rpm*/
	double getWheelVelocity();
	/**@return This distance traveled by the wheel in meters this value can be plugged straight into the module position class*/
	double getWheelPosition();
	/**This function needs to be called periodically to ensure that the sim is periodically updated and this function also logs data.*/
	void update();
}
