package frc.robot.subsystem.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;

public class SwerveModule {
    private final SwerveModuleIO module;

    private final PIDController driveController;
    private final PIDController steerController;
    private final SimpleMotorFeedforward driveFF;

    private SwerveModuleState state;
    private SwerveModulePosition position;

    private SpeedMode mode;

    /*** Target speed of the motor in rpm*/
    double targetSpeed;

    public SwerveModule(SwerveModuleDetails details)
    {
        driveController = new PIDController(0,0,0);
        driveFF = new SimpleMotorFeedforward(0,0);

        steerController = new PIDController(0,0,0);

        position = new SwerveModulePosition();

        state = new SwerveModuleState();

        if (Robot.isSimulation())
        {
            module = new SwerveModuleSim(details);
        }else {
            module = new SwerveModuleRev(details);
        }

    }
    void periodic(){
        module.update();


        module.setDriveVoltage(
                driveController.calculate(module.getWheelVelocity(), state.speedMetersPerSecond * (Math.PI * 4))

        );
    }

    void setState(SwerveModuleState state)
    {
        this.state = state;
    }

    void resetModulePositions()
    {
        position = new SwerveModulePosition();
    }

    void setSpeedMode(SpeedMode mode)
    {
        this.mode = mode;
    }
    public SwerveModulePosition getModulePosition()
    {
        return position;
    }

    SwerveModuleState getState()
    {
        return state;
    }
}
