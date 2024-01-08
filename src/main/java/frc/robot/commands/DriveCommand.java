package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Deadband;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystem.drivetrain.DrivetrainConstants;
import frc.robot.subsystem.drivetrain.DrivetrainMode;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystem.drivetrain.SpeedMode;
import org.littletonrobotics.junction.Logger;

/**
 * The drive train command is meant to handle drivetrain inputs in telop
 */
public class DriveCommand extends Command {

	private final Deadband xDeadband = new Deadband(.1);
	private final Deadband yDeadband = new Deadband(.1);
	private final Deadband thetaDeadband = new Deadband(.1);

	/**
	 * The Slew Rate Limiter is applied when the input is between -1 and 1.
	 * If the slew rate is 1 that means that it will take 1 second to get to 1 if it was 2 this would mean it would take 1 second to get to 2
	 */
	private final SlewRateLimiter xSlewRate = new SlewRateLimiter(2,-2,0);
	private final SlewRateLimiter ySlewRate = new SlewRateLimiter(2,-2,0);
	private final SlewRateLimiter thetaSlewRate = new SlewRateLimiter(1,-1,0);


	public DriveCommand() {
		addRequirements(DrivetrainSubsystem.getInstance());
	}

	@Override
	public void execute() {

		// Joystick input

		double x = 0;
		double y = 0;
		double theta = 0;

		x = IO.getJoystickValue(Controls.drive_x).get();
		y = IO.getJoystickValue(Controls.drive_y).get();
		theta = IO.getJoystickValue(Controls.drive_theta).get();


		x = xDeadband.apply(x);
		y = yDeadband.apply(y);
		theta = thetaDeadband.apply(theta);
		Logger.recordOutput("RawY", y);

		if (RobotState.isEnabled())
		{

//			x = xSlewRate.calculate(x);
//			y = ySlewRate.calculate(y);
//			theta = thetaSlewRate.calculate(theta);
		}


		x = Math.pow(x,3);
		y = Math.pow(y,3);
		theta = Math.pow(theta,3);
		Logger.recordOutput("LimitY", y);

		if (DrivetrainSubsystem.getInstance().getSpeedMode() == SpeedMode.normal)
		{
			x = -x * DrivetrainConstants.NORMAL_SPEED;
			y = -y * DrivetrainConstants.NORMAL_SPEED;
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

		if (IO.getJoystickValue(Controls.autoTurnPOV).get().intValue() != -1)
		{
			DrivetrainSubsystem.getInstance().setMode(DrivetrainMode.telop_auto_turn);
		}
	}
}
