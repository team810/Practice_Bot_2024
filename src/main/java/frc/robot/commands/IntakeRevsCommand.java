package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeSubsystem;


public class IntakeRevsCommand extends Command {

    public IntakeRevsCommand() {

        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setState(IntakeSubsystem.IntakeState.revs);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setState(IntakeSubsystem.IntakeState.off);
    }
}
