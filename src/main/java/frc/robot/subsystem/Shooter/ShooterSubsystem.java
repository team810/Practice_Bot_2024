package frc.robot.subsystem.Shooter;


import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem INSTANCE;

    private final ShooterIO shooter;

    private double topTargetRPM;
    private double bottomTargetRPM;
    private double topVoltage;
    private double bottomVoltage;

    private final PIDController topController;
    private final PIDController bottomController;
    private ShooterMode shooterMode;





    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null)
        {
            INSTANCE = new ShooterSubsystem();
        }
        return INSTANCE;
    }

    private ShooterSubsystem() {

        topController = new PIDController(0,0,0);
        bottomController = new PIDController(0,0,0);

        shooter = new ShooterNEO();

        topController.setP(ShooterConstants.TOP_CONTROLLER.kP);
        topController.setI(ShooterConstants.TOP_CONTROLLER.kI);
        topController.setD(ShooterConstants.TOP_CONTROLLER.kD);

        bottomController.setP(ShooterConstants.BOTTOM_CONTROLLER.kP);
        bottomController.setI(ShooterConstants.BOTTOM_CONTROLLER.kI);
        bottomController.setD(ShooterConstants.BOTTOM_CONTROLLER.kD);

        topController.setTolerance(0.1);
        bottomController.setTolerance(0.1);

        shooterMode = ShooterMode.Off;



    }

    public ShooterMode getShooterMode() {
        return shooterMode;
    }

    public void setShooterMode(ShooterMode mode) {
        shooterMode = mode;
    }

    public void periodic(){
        if(RobotState.isEnabled()){

            switch (shooterMode) {
                case On -> {
                    topTargetRPM = ShooterConstants.SHOOTER_MAX_RPM;
                    bottomTargetRPM = ShooterConstants.SHOOTER_MAX_RPM;

                    topVoltage = topController.calculate(
                            shooter.getTopRPM(),
                            topTargetRPM
                    );

                    bottomVoltage = bottomController.calculate(
                            shooter.getBottomRPM(),
                            bottomTargetRPM
                    );

                    shooter.setTopVoltage(topVoltage);
                    shooter.setBottomVoltage(bottomVoltage);
                }
                case Off -> {
                    topTargetRPM = 0;
                    bottomTargetRPM = 0;

                    topController.reset();
                    bottomController.reset();

                    shooter.setTopVoltage(0);
                    shooter.setBottomVoltage(0);
                    System.out.println("Off");
                }
            }

        }

        shooter.update();
    }





}

