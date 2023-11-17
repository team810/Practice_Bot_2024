package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.IO;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;

public class RobotContainer
{

    public RobotContainer()
    {
        DriverStation.silenceJoystickConnectionWarning(true);
        IO.Initialize();

        DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand());
        XboxController controller = new XboxController(0);
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public Command getAutonomousCommand()
    {
        return null;
    }
}
