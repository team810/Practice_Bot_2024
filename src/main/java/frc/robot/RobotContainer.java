package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.IO;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;

public class RobotContainer
{
    public RobotContainer()
    {
        IO.Initialize();

        DrivetrainSubsystem.getInstance();
    }

    public Command getAutonomousCommand()
    {
        return null;
    }
}
