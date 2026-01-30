package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    public PhotonVision(String name) {
        camera = new PhotonCamera(name);
        latestResult = camera.getLatestResult();
    }

    public boolean hasTargets() {
        return latestResult != null && latestResult.hasTargets();
    }

    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }

    public double getYaw() {
        if (hasTargets()) {
            double yaw = camera.getLatestResult().getBestTarget().getYaw(); 
            return yaw; 
        }
        return 0.0; 
    }



    @Override public void periodic() {
        latestResult = camera.getLatestResult();
        SmartDashboard.putBoolean("Has Target", latestResult.hasTargets());
    }
}
