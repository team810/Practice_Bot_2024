package frc.robot.subsystem.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

interface CameraIO {

    public void setCameraMode(CameraMode mode);
    public CameraMode getCurrentCameraMode();

    public PhotonPipelineResult getResults();

    public void update();

    public PhotonCamera getCamera();
}
