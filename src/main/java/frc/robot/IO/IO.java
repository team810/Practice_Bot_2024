package frc.robot.IO;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.StadiaController;

import java.util.HashMap;
import java.util.function.Supplier;

public abstract class IO {
    private static final XboxController primary = new StadiaController(0);
    private static final XboxController secondary = new StadiaController(1);

    private static final HashMap<Controls,Supplier<Double>> controlsJoystick = new HashMap<>();
    private static final HashMap<Controls,Supplier<Boolean>> controlsButtons = new HashMap<>();

    public static void Initialize()
    {
        controlsJoystick.put(Controls.drive_x, primary::getLeftX);
        controlsJoystick.put(Controls.drive_y, primary::getLeftY);
        controlsJoystick.put(Controls.drive_theta, primary::getRightX);
        controlsJoystick.put(Controls.autoTurnPOV, () -> (double) primary.getPOV());

        controlsButtons.put(Controls.reset_gyro, primary::getLeftBumper);
        controlsButtons.put(Controls.slowMode, primary::getRightBumper);
        controlsButtons.put(Controls.normalMode, () -> (.75 > primary.getRightTriggerAxis()));

        controlsButtons.put(Controls.intakeFwd, secondary::getAButton);
        controlsButtons.put(Controls.intakeRevs, secondary::getYButton);
    }

    public static Supplier<Double> getJoystickValue(Controls control)
    {
        return controlsJoystick.get(control);
    }

    public static Supplier<Boolean> getButtonValue(Controls control)
    {
        return controlsButtons.get(control);
    }
}
