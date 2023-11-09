package frc.robot.subsystem.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

	void setState(SwerveModuleState state);
	void resetModulePositions();
	void setSpeedMode(SpeedMode mode);

	SwerveModulePosition getModulePosition();
	SwerveModuleState getState();

	void periodic();
}
