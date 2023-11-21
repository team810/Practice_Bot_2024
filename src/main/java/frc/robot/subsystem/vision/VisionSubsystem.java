package frc.robot.subsystem.vision;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final static VisionSubsystem INSTANCE = new VisionSubsystem();
    public static VisionSubsystem getInstance() {
        return INSTANCE;
    }

    UsbCamera usbCam1;
    UsbCamera usbCam2;

    private VisionSubsystem() {
        usbCam1 = CameraServer.startAutomaticCapture(0);
        usbCam2 = CameraServer.startAutomaticCapture(1);

    }

    @Override
    public void periodic() {

    }
}

