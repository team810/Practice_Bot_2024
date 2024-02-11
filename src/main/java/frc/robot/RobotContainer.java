package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.IO;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;

public class RobotContainer {

    public RobotContainer()
    {

        DriverStation.silenceJoystickConnectionWarning(true);

        IO.Initialize();

        DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand());

    }

    public Command getAutonomousCommand()
    {
        return null;
    }
}
