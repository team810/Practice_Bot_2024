package frc.robot.subsystem.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.GlobalConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO module;

    private final PIDController driveController;
    private final PIDController steerController;
    private final SimpleMotorFeedforward driveFF;

    private SwerveModuleState state;
    private SwerveModulePosition position;

    private SpeedMode mode;
    private final SwerveModuleDetails details;

    public SwerveModule(SwerveModuleDetails details)
    {
        driveController = new PIDController(0.035,0,0);
        driveController.setTolerance(100,100);
        driveFF = new SimpleMotorFeedforward(.116970,.133240);

        steerController = new PIDController(3,3,1);
        steerController.setTolerance(.01);
        steerController.enableContinuousInput(-Math.PI, Math.PI);

        position = new SwerveModulePosition();

        state = new SwerveModuleState();

        if (Robot.isSimulation())
        {
            module = new SwerveModuleSim(details);
        }else {
            module = new SwerveModuleRev(details);
        }

        this.details = details;
    }
    void periodic(){
        module.update();

        double speedOfMotorRPM = state.speedMetersPerSecond;
        
        if (mode == SpeedMode.slow)
        {
            speedOfMotorRPM = speedOfMotorRPM / DrivetrainConstants.SLOW_SPEED;
        } else if (mode == SpeedMode.normal) {
            speedOfMotorRPM = speedOfMotorRPM / DrivetrainConstants.NORMAL_SPEED;
        }
        speedOfMotorRPM = speedOfMotorRPM * GlobalConstants.NEO_MAX_RPM;

        module.setDriveVoltage(
                driveController.calculate(module.getWheelVelocity(), speedOfMotorRPM) +
                driveFF.calculate(speedOfMotorRPM)
        );
        module.setSteerVoltage(
                steerController.calculate(MathUtil.angleModulus(module.getWheelAngle().getRadians()), MathUtil.angleModulus(state.angle.getRadians()))
        );


        Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/TargetVelocity", speedOfMotorRPM);
        Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/TargetAngle", state.angle.getRadians());
        Logger.getInstance().recordOutput("Drivetrain/" + details.module.name() + "/AtAngleSetpoint", steerController.atSetpoint());
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
        double speedOfWheel = module.getWheelVelocity();
        speedOfWheel = speedOfWheel / GlobalConstants.NEO_MAX_RPM;

        if (mode == SpeedMode.normal)
        {
            speedOfWheel = speedOfWheel * DrivetrainConstants.NORMAL_SPEED;
        } else if (mode == SpeedMode.slow) {
            speedOfWheel = speedOfWheel * DrivetrainConstants.SLOW_SPEED;
        }

        return new SwerveModuleState(speedOfWheel, module.getWheelAngle());
    }
}
