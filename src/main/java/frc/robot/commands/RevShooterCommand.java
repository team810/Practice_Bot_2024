package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Shooter.ShooterMode;
import frc.robot.subsystem.Shooter.ShooterSubsystem;


public class RevShooterCommand extends Command {

    public RevShooterCommand() {

        addRequirements(ShooterSubsystem.getInstance());
    }

    @Override
    public void initialize() {

       ShooterSubsystem.getInstance().setShooterMode(ShooterMode.On);

    }

    @Override
    public void end(boolean interrupted) {

       ShooterSubsystem.getInstance().setShooterMode(ShooterMode.Off);

    }
}
