package frc.robot.subsystem.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;

class LimelightSim implements CameraIO{

    private final SimVisionSystem simVisionSystem;


    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;

    private CameraMode mode;

    public LimelightSim(Transform3d translation, String cameraName, CameraMode mode, AprilTagFieldLayout fieldLayout)
    {
        this.fieldLayout = fieldLayout;


        simVisionSystem = new SimVisionSystem(
                cameraName,
                90,
                translation,
                1000,
                640,
                480,
                10
        );

        camera = new PhotonCamera(cameraName);
        setCameraMode(mode);
    }
    @Override
    public void setCameraMode(CameraMode mode) {
        this.mode = mode;

        switch (this.mode)
        {
            case Limelight:
                simVisionSystem.clearVisionTargets();
                break;
            case ColoredShape:
                simVisionSystem.clearVisionTargets();
                break;
            case AprilTag:
                simVisionSystem.clearVisionTargets();
                simVisionSystem.addVisionTargets(fieldLayout);
                break;
            default:
                throw new RuntimeException("Camera Mode Switch statement not all case accounted for");
        }
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
        simVisionSystem.processFrame(DrivetrainSubsystem.getInstance().getPose());
    }

    @Override
    public PhotonCamera getCamera() {
        return camera;
    }
}
