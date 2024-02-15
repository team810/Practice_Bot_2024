package frc.robot.subsystem.Shooter;


import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import static frc.robot.subsystem.Shooter.ShooterConstants.TOP_MOTOR_ID;

public class ShooterNEO implements ShooterIO{

    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;
    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;
    private double topVoltage;
    private double bottomVoltage;

    public ShooterNEO(){
        topMotor = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        bottomMotor = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

        topMotor.setSmartCurrentLimit(40);
        bottomMotor.setSmartCurrentLimit(40);

        topMotor.enableVoltageCompensation(12);
        bottomMotor.enableVoltageCompensation(12);

        topMotor.clearFaults();
        bottomMotor.clearFaults();

        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();

        topVoltage = 0;
        bottomVoltage = 0;



    }




    @Override
    public void setTopVoltage(double voltage) {
        topVoltage = voltage;
    }

    @Override
    public double getTopVoltage() {
        return topVoltage;
    }

    @Override
    public void setBottomVoltage(double voltage) {
        bottomVoltage = voltage;
    }

    @Override
    public double getBottomVoltage() {
        return bottomVoltage;
    }

    @Override
    public double getTopRPM() {
        return topEncoder.getVelocity();
    }

    @Override
    public double getBottomRPM() {
        return bottomEncoder.getVelocity();
    }
}
