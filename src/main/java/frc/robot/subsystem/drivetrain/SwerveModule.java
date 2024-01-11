package frc.robot.subsystem.drivetrain;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

class SwerveModule {
    private final SwerveModuleIO module;

    private final PIDController driveController;
    private final PIDController steerController;

    private SwerveModuleState state;
    private SwerveModulePosition position;

    private SpeedMode mode;
    private final SwerveModuleDetails details;

    public SwerveModule(SwerveModuleDetails details)
    {
        driveController = new PIDController(0,0,0, Robot.defaultPeriodSecs);
        steerController = new PIDController(0,0,0, Robot.defaultPeriodSecs);

        driveController.setTolerance(10);
        steerController.setTolerance(0);

        if (Robot.isReal())
        {
            driveController.setP(DrivetrainConstants.DRIVE_CONTROLLER_REAL.kP);
            driveController.setP(DrivetrainConstants.DRIVE_CONTROLLER_REAL.kP);
            driveController.setI(DrivetrainConstants.DRIVE_CONTROLLER_REAL.kI);
            driveController.setD(DrivetrainConstants.DRIVE_CONTROLLER_REAL.kD);

            steerController.setP(DrivetrainConstants.STEER_CONTROLLER_REAL.kP);
            steerController.setI(DrivetrainConstants.STEER_CONTROLLER_REAL.kI);
            steerController.setD(DrivetrainConstants.STEER_CONTROLLER_REAL.kD);

            steerController.enableContinuousInput(-Math.PI, Math.PI);
            steerController.setTolerance(.01);

        } else if (Robot.isSimulation()) {

            driveController.setP(DrivetrainConstants.DRIVE_CONTROLLER_SIM.kP);
            driveController.setI(DrivetrainConstants.DRIVE_CONTROLLER_SIM.kI);
            driveController.setD(DrivetrainConstants.DRIVE_CONTROLLER_SIM.kD);

            steerController.setP(DrivetrainConstants.STEER_CONTROLLER_SIM.kP);
            steerController.setI(DrivetrainConstants.STEER_CONTROLLER_SIM.kI);
            steerController.setD(DrivetrainConstants.DRIVE_CONTROLLER_SIM.kD);

            steerController.enableContinuousInput(-Math.PI, Math.PI);
            steerController.setTolerance(.01);
        }else{
            throw new RuntimeException(
                    "The PID controls for both the drive controller " +
                    "and the steer controllers are not getting configured, " +
                    "how is the robot not real or simulated"
            );
        }

        position = new SwerveModulePosition();

        state = new SwerveModuleState();

        if (Robot.isSimulation())
        {
            module = new SwerveModuleSim(details);
        }else {
            module = new SwerveModuleRev(details);
        }

        this.details = details;


        module.setState(new SwerveModuleState(0,new Rotation2d()));
    }

    public void setIdleMode(CANSparkMax.IdleMode mode)
    {
        module.setIdleMode(mode);
    }

    void periodic(){
        module.setState(state);

        double speedOfMotorRPM = (state.speedMetersPerSecond / DrivetrainConstants.DISTANCE_PER_REVOLUTION) * 60 * DrivetrainConstants.GEAR_REDUCTION_DRIVE;

        module.setDriveVoltage(
                driveController.calculate(module.getWheelVelocity(), speedOfMotorRPM)
        );
        module.setSteerVoltage(
                steerController.calculate(module.getWheelAngle().getRadians(), MathUtil.angleModulus(state.angle.getRadians()))
        );

        module.update();
        Logger.recordOutput("Drivetrain/" + details.module.name() + "/TargetVelocity", speedOfMotorRPM);
        Logger.recordOutput("Drivetrain/" + details.module.name() + "/TargetAngle", state.angle.getRadians());
        Logger.recordOutput("Drivetrain/" + details.module.name() + "/AtAngleSetpoint", steerController.atSetpoint());

        if (RobotState.isDisabled())
        {
            steerController.reset();

            driveController.reset();

        }

        position = new SwerveModulePosition(module.getWheelPosition(),module.getWheelAngle());

    }

    void setState(SwerveModuleState state)
    {
        this.state = state;
    }

    void resetModulePositions()
    {
        module.resetPosition();
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
        speedOfWheel = (((speedOfWheel / DrivetrainConstants.GEAR_REDUCTION_DRIVE) / 60)) * DrivetrainConstants.DISTANCE_PER_REVOLUTION;

        return new SwerveModuleState(speedOfWheel, module.getWheelAngle());
    }
}
