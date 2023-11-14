package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Deadband;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystem.drivetrain.DrivetrainConstants;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystem.drivetrain.SpeedMode;


public class DriveCommand extends CommandBase {

    private final Deadband xDeadband = new Deadband(.05,0);
	private final Deadband yDeadband = new Deadband(.05,0);
    private final Deadband thetaDeadband = new Deadband(.05,0);

	public DriveCommand() {
		addRequirements(DrivetrainSubsystem.getInstance());
	}

	@Override
	public void execute() {

		// Joystick input

		double x = 0;
		double y = 0;
		double theta = 0;

		x = -IO.getJoystickValue(Controls.drive_x).get();
		y = -IO.getJoystickValue(Controls.drive_y).get();
		theta = -IO.getJoystickValue(Controls.drive_theta).get();

		x = xDeadband.apply(x);
		y = yDeadband.apply(y);
		theta = thetaDeadband.apply(theta);

        x = Math.pow(x,3);
        y = Math.pow(y,3);
        theta = Math.pow(theta,3);

		if (DrivetrainSubsystem.getInstance().getSpeedMode() == SpeedMode.normal)
		{
			x = x * DrivetrainConstants.NORMAL_SPEED;
			y = y * DrivetrainConstants.NORMAL_SPEED;
			theta = theta * DrivetrainConstants.NORMAL_SPEED;

			x = MathUtil.clamp(x, -DrivetrainConstants.NORMAL_SPEED, DrivetrainConstants.NORMAL_SPEED);
			y = MathUtil.clamp(y, -DrivetrainConstants.NORMAL_SPEED, DrivetrainConstants.NORMAL_SPEED);
			theta = MathUtil.clamp(theta, -DrivetrainConstants.NORMAL_SPEED, DrivetrainConstants.NORMAL_SPEED);
		}
		if (DrivetrainSubsystem.getInstance().getSpeedMode() == SpeedMode.slow)
		{
			x = x * DrivetrainConstants.SLOW_SPEED;
			y = y * DrivetrainConstants.SLOW_SPEED;
			theta = theta * DrivetrainConstants.SLOW_SPEED;

			x = MathUtil.clamp(x, -DrivetrainConstants.SLOW_SPEED, DrivetrainConstants.SLOW_SPEED);
			y = MathUtil.clamp(y, -DrivetrainConstants.SLOW_SPEED, DrivetrainConstants.SLOW_SPEED);
			theta = MathUtil.clamp(theta, -DrivetrainConstants.SLOW_SPEED, DrivetrainConstants.SLOW_SPEED);
		}


        DrivetrainSubsystem.getInstance().setTargetTeleopSpeeds(
                x,
                y,
                theta
        );

		// Button Input

		if (IO.getButtonValue(Controls.reset_gyro).get())
		{
			DrivetrainSubsystem.getInstance().zeroGyro();
		}

		if (IO.getButtonValue(Controls.slowMode).get())
		{
			DrivetrainSubsystem.getInstance().setSpeedMode(SpeedMode.slow);
		}

		if (IO.getButtonValue(Controls.normalMode).get())
		{
			DrivetrainSubsystem.getInstance().setSpeedMode(SpeedMode.normal);
		}

	}
}
