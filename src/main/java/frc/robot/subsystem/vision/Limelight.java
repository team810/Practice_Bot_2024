package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Limelight implements CameraIO
{
    private final PhotonCamera limelight;
    private final String cameraName;
    private CameraMode mode;
    /**
     * @param cameraName This is the name of the camera that is assigned in photon vision
     * @param translation This is the translation of the robot to the camera
     * @param defaultMode This is the starting mode that the camera will be in
     */
    public Limelight(
            String cameraName,
            Transform3d translation,
            CameraMode defaultMode
    )
    {
        limelight = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        setCameraMode(defaultMode);
    }
    @Override
    public void setCameraMode(CameraMode mode) {
        this.mode = mode;

        switch (this.mode)
        {
            case Limelight:
                limelight.setPipelineIndex(VisionConstants.LIME_LIGHT_INDEX);
                break;
            case ColoredShape:
                limelight.setPipelineIndex(VisionConstants.COLORED_SHAPE_INDEX);
                break;
            case AprilTag:
                limelight.setPipelineIndex(VisionConstants.APRIL_TAG_INDEX);
                break;
        }
    }

    @Override
    public PhotonPipelineResult getResults() {
        return limelight.getLatestResult();
    }
    @Override
    public void update() {
        Logger.getInstance().recordOutput("Camera/" + cameraName + "/IsConnected", limelight.isConnected());
        Logger.getInstance().recordOutput("Camera/" + cameraName + "/Mode", this.mode.toString());
    }

    @Override
    public PhotonCamera getCamera() {
        return limelight;
    }

    @Override
    public CameraMode getCurrentCameraMode() {
        return mode;
    }
}
