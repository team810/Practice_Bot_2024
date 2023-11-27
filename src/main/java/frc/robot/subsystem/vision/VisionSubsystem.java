package frc.robot.subsystem.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final static VisionSubsystem INSTANCE = new VisionSubsystem();
    public static VisionSubsystem getInstance() {
        return INSTANCE;
    }

    private final CameraIO limelight;

    private final UsbCamera usbCam1;
    private final UsbCamera usbCam2;

    private CameraMode mode;
    private final AprilTagFieldLayout fieldLayout;

    private final PhotonPoseEstimator estimator;

    private VisionSubsystem() {

        try {
            fieldLayout = new AprilTagFieldLayout("src/main/deploy/2023-chargedup.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        mode = CameraMode.AprilTag;

        usbCam1 = CameraServer.startAutomaticCapture(0);
        usbCam2 = CameraServer.startAutomaticCapture(1);

        if (Robot.isSimulation())
        {
            limelight = new LimelightSim(VisionConstants.LIME_LIGHT_POSITION, "limelight",CameraMode.AprilTag, fieldLayout);

        } else if (Robot.isReal()) {

            limelight = new Limelight("limelight", VisionConstants.LIME_LIGHT_POSITION, CameraMode.AprilTag);
        }else{
            throw new RuntimeException("How are you not in sim or in real");
        }

        estimator = new PhotonPoseEstimator(getFieldLayout(), PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, limelight.getCamera(), VisionConstants.LIME_LIGHT_POSITION);
    }

    public void setMode(CameraMode mode) {
        this.mode = mode;

        limelight.setCameraMode(mode);
    }

    @Override
    public void periodic() {
        limelight.update();
        for (int i = 0; i < limelight.getResults().getTargets().size(); i++) {
            Pose3d targetPose = new Pose3d(DrivetrainSubsystem.getInstance().getPose()).transformBy(limelight.getResults().targets.get(i).getBestCameraToTarget());
            Logger.getInstance().recordOutput("Vision/AprilTag/" + i , targetPose);
        }


        Optional<EstimatedRobotPose> updated = estimator.update();
        if (updated.isPresent())
        {
            if (limelight.getResults().hasTargets())
            {
                if (limelight.getResults().targets.get(0).getArea() > 800)
                {

                    Logger.getInstance().recordOutput("VisionPose", updated.get().estimatedPose);
                    DrivetrainSubsystem.getInstance().resetOdometry(updated.get().estimatedPose.toPose2d());
                }else {
                    Logger.getInstance().recordOutput("VisionPose", new Pose3d(0,0,0,new Rotation3d()));
                }
                Logger.getInstance().recordOutput("TargetSize", limelight.getResults().targets.get(0).getArea());
            } else {
//                Logger.getInstance().recordOutput("VisionPose", new Pose3d(DrivetrainSubsystem.getInstance().getPose()));
                Logger.getInstance().recordOutput("VisionPose", new Pose3d(0,0,0,new Rotation3d()));
            }
        }else {
//            Logger.getInstance().recordOutput("VisionPose", DrivetrainSubsystem.getInstance().getPose());
            Logger.getInstance().recordOutput("VisionPose", new Pose3d(0,0,0,new Rotation3d()));
        }

    }


    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public CameraMode getMode() {
        return mode;
    }
}

