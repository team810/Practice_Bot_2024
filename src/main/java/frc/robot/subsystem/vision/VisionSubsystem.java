package frc.robot.subsystem.vision;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;
import org.photonvision.SimVisionSystem;

public class VisionSubsystem extends SubsystemBase {
    private final static VisionSubsystem INSTANCE = new VisionSubsystem();
    public static VisionSubsystem getInstance() {
        return INSTANCE;
    }

    private final Limelight limelight;

    private final UsbCamera usbCam1;
    private final UsbCamera usbCam2;

    private final SimVisionSystem simVisionSystem;

    private CameraMode mode;

    private VisionSubsystem() {

        mode = CameraMode.AprilTag;

        if (Robot.isSimulation())
        {
            simVisionSystem = new SimVisionSystem(
                    "limelight",
                    90,
                    VisionConstants.LIME_LIGHT_POSITION,
                    9000,
                    640,
                    480,
                    10);

            switch (mode) {
                case Limelight:
                    break;
                case ColoredShape:
                    break;
                case AprilTag:
                    break;
            }
        }else{
            simVisionSystem = null;
        }

        usbCam1 = CameraServer.startAutomaticCapture(0);
        usbCam2 = CameraServer.startAutomaticCapture(1);

        limelight = new Limelight("limelight", VisionConstants.LIME_LIGHT_POSITION, CameraMode.AprilTag);
    }

    public void setMode(CameraMode mode) {
        this.mode = mode;

        switch (mode) {
            case Limelight:
                break;
            case ColoredShape:
                break;
            case AprilTag:
                break;
        }
    }

    @Override
    public void periodic() {
        simVisionSystem.processFrame(DrivetrainSubsystem.getInstance().getPose());
    }

    public CameraMode getMode() {
        return mode;
    }
}

