package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.*;
import frc.robot.subsystem.Shooter.ShooterSubsystem;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystem.intake.IntakeSubsystem;

public class RobotContainer {

    public RobotContainer()
    {

        DriverStation.silenceJoystickConnectionWarning(true);

        IO.Initialize();

        DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand());
        IntakeSubsystem.getInstance();
        ShooterSubsystem.getInstance();

        new Trigger(() -> IO.getButtonValue(Controls.intakeFwd).get()).
                whileTrue(new IntakeFwdCommand());
        new Trigger(() -> IO.getButtonValue(Controls.intakeRevs).get()).
                whileTrue(new IntakeRevsCommand());
        new Trigger(() -> IO.getButtonValue(Controls.intakeFire).get()).
                whileTrue(new FireCommand());

        new Trigger(() -> IO.getButtonValue(Controls.intakeFire).get()).
                whileTrue(new FireCommand());
        new Trigger(() -> IO.getButtonValue(Controls.shooterFire).get()).
                whileTrue(new RevShooterCommand());
    }

    public Command getAutonomousCommand()
    {
        return null;
    }
}
