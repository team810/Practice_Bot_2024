package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {

    public static final int LIME_LIGHT_INDEX = 0;
    public static final int APRIL_TAG_INDEX = 1;
    public static final int COLORED_SHAPE_INDEX = 2;

    public static final Transform3d LIME_LIGHT_POSITION = new Transform3d(new Translation3d(), new Rotation3d());
}
