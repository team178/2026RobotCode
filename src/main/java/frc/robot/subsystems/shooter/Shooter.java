package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIOL;
    private final ShooterIO shooterIOM;
    private final ShooterIO shooterIOR;
    private final ShooterIO feederIO;

    public Shooter(ShooterIO shooterIOL, ShooterIO shooterIOM, ShooterIO shooterIOR, ShooterIO feederIO) {
        this.shooterIOL = shooterIOL;
        this.shooterIOM = shooterIOM;
        this.shooterIOR = shooterIOR;
        this.feederIO = feederIO;
    }

    private double shooterMotorRegressionDistanceToVelocityRadPerSec(double distanceMeters) {
        return distanceMeters; // TODO update regression
    }

    private double feederMotorRegressionDistanceToVelocityRadPerSec(double distanceMeters) {
        return distanceMeters; // TODO update regression
    }

    private void shootWithDistance(double distanceMeters) {
        double shooterVelocityRadPerSec = shooterMotorRegressionDistanceToVelocityRadPerSec(distanceMeters);
        double feederVelocityRadPerSec = feederMotorRegressionDistanceToVelocityRadPerSec(distanceMeters);

        shooterIOL.setVelocityClosedLoop(shooterVelocityRadPerSec);
        shooterIOM.setVelocityClosedLoop(shooterVelocityRadPerSec);
        shooterIOR.setVelocityClosedLoop(shooterVelocityRadPerSec);
        feederIO.setVelocityClosedLoop(feederVelocityRadPerSec);
    }

    public Command runStopShooter() {
        return runOnce(() -> {
            shooterIOL.setOpenLoop(0);
            shooterIOM.setOpenLoop(0);
            shooterIOR.setOpenLoop(0);
            feederIO.setOpenLoop(0);
        });
    }

    public Command runShootMaxSpeed() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            feederIO.setVelocityClosedLoop(ShooterConstants.feederMotorMaxSpeed);
        });
    }

    public Command runShooterIdle() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            feederIO.setOpenLoop(0);
        });
    }

    public Command runShootAtHub(Supplier<Pose2d> poseSupplier) {
        return run(() -> {
            Pose2d robotPose = poseSupplier.get();
//            Pose2d hubPose = FieldConstants.getHubCenter();
            Pose2d hubPose = new Pose2d();

            double hubDistance = robotPose.getTranslation().getDistance(hubPose.getTranslation());

            shootWithDistance(hubDistance);
        });
    }
}
