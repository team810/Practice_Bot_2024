package frc.robot.IO;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.StadiaController;

import java.util.HashMap;
import java.util.function.Supplier;

public class IO {
    private static final XboxController primary = new XboxController(0);
    private static final StadiaController secondary = new StadiaController(1);

    private final HashMap<Supplier<Double>, Controls> controlsJoystick = new HashMap<>();
    private final HashMap<Supplier<Boolean>, Controls> controlsButtons = new HashMap<>();

    public static void Initialize()
    {

    }

    public Supplier<Double> getJoystickValue(Controls control)
    {
        return null;
    }

    public Supplier<Boolean> getButtonValue(Controls control)
    {
        return null;
    }
}
