package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeFwdCommand;
import frc.robot.commands.IntakeRevsCommand;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystem.intake.IntakeSubsystem;

public class RobotContainer {

    public RobotContainer()
    {

        DriverStation.silenceJoystickConnectionWarning(true);

        IO.Initialize();

        DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand());
        IntakeSubsystem.getInstance();

        new Trigger(() -> IO.getButtonValue(Controls.intakeFwd).get()).
                whileTrue(new IntakeFwdCommand());
        new Trigger(() -> IO.getButtonValue(Controls.intakeRevs).get()).
                whileTrue(new IntakeRevsCommand());
    }

    public Command getAutonomousCommand()
    {
        return null;
    }
}
