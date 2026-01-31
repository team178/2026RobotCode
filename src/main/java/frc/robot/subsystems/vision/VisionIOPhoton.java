package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.Constants.VisionConstants;

public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;

    private final PhotonPoseEstimator photonEstimator;

    public VisionIOPhoton(String name, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;

        photonEstimator = new PhotonPoseEstimator(VisionConstants.aprilTagLayout, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs visionIOInputs) {
        var result = camera.getLatestResult();
        periodic();

        boolean hasTargets = result.hasTargets();

        PhotonTrackedTarget[] targets = result.getTargets().toArray(PhotonTrackedTarget[]::new);

        for (PhotonTrackedTarget target : targets) {
            // double yaw = target.getYaw();
            // double pitch = target.getPitch();
            // double area = target.getArea();
            // double skew = target.getSkew();
            // // Transform2d pose = target.getCameraToTarget();
            // // List<TargetCorner> corners = target.getCorners();

            // int targetID = target.getFiducialId();
            // double poseAmbiguity = target.getPoseAmbiguity();
            // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
        }
        
    }

    private void periodic() {
        
    }
}
