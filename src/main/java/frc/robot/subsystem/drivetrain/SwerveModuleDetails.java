package frc.robot.subsystem.drivetrain;

public class SwerveModuleDetails {
	public int driveID;
	public int steerID;
	public int encoderID;
	public double encoderOffset;
	public SwerveModuleEnum module;

	public SwerveModuleDetails(int mDriveID, int mSteerID, int mEncoderID, double mEncoderOffset, SwerveModuleEnum mModule) {
		driveID = mDriveID;
		steerID = mSteerID;
		encoderID = mEncoderID;
		encoderOffset = mEncoderOffset;
		module = mModule;

	}
}
