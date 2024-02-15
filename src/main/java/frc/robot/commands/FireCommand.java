package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeSubsystem;


public class FireCommand extends Command {

    public FireCommand() {

        addRequirements(IntakeSubsystem.getInstance());

    }

    @Override
    public void initialize() {

        IntakeSubsystem.getInstance().setState(IntakeSubsystem.IntakeState.fire);

    }



    @Override
    public void end(boolean interrupted) {

        IntakeSubsystem.getInstance().setState(IntakeSubsystem.IntakeState.off);

    }
}
