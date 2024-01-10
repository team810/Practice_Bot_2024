package frc.robot.subsystem.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

class LimelightSim implements CameraIO{



    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;

    private CameraMode mode;

    public LimelightSim(Transform3d translation, String cameraName, CameraMode mode, AprilTagFieldLayout fieldLayout)
    {
        this.fieldLayout = fieldLayout;




        camera = new PhotonCamera(cameraName);
        setCameraMode(mode);
    }
    @Override
    public void setCameraMode(CameraMode mode) {
        this.mode = mode;
    }

    @Override
    public CameraMode getCurrentCameraMode() {
        return null;
    }

    @Override
    public PhotonPipelineResult getResults() {
        return camera.getLatestResult();
    }

    @Override
    public void update() {
    }

    @Override
    public PhotonCamera getCamera() {
        return camera;
    }
}
