package frc.lib.check;

import com.revrobotics.CANSparkMax;

public class NeoSelfCheek {
    public static void FaultCheek(CANSparkMax motor)
    {

        if (motor.getFault(CANSparkMax.FaultID.kSensorFault))
        {
            System.out.println("WARNING: SENSOR FAULT ON MOTOR " + motor.getDeviceId() + " THIS MOTOR WILL NOT RUN IF IT A NEO OR NEO 550");
        }

    }
}
