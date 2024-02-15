package frc.robot.subsystem.Shooter;

public interface ShooterIO {

    void setTopVoltage(double voltage);
    double getTopVoltage();
    void setBottomVoltage(double voltage);
    double getBottomVoltage();

    double getTopRPM();
    double getBottomRPM();

}
