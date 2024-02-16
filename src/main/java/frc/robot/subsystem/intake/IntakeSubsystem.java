package frc.robot.subsystem.intake;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem INSTANCE;
    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null)
        {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }

    private final CANSparkMax bottomMotor;
    private final CANSparkMax topMotor;

    private IntakeState state;
    private static final double INTAKE_SPEED = .5;


    private IntakeSubsystem() {
        bottomMotor = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless);
        topMotor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);

        topMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        topMotor.setInverted(true);
        state = IntakeState.off;
    }

    @Override
    public void periodic()
    {
        switch (state)
        {
            case fwd -> {
                topMotor.set(INTAKE_SPEED);
                bottomMotor.set(INTAKE_SPEED);
            }
            case revs -> {
                topMotor.set(-INTAKE_SPEED);
                bottomMotor.set(-INTAKE_SPEED);
            }
            case off -> {
                topMotor.set(0);
                bottomMotor.set(0);
            }
            case fire ->{
                topMotor.set(.7);
                bottomMotor.set(.7);
            }
        }
    }

    public IntakeState getState() {
        return state;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public enum IntakeState
    {
        fwd,
        revs,
        off,
        fire
    }

}

