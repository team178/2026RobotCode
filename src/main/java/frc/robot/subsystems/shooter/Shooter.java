package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.HubShootLUT;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIOL;
    private final ShooterIO shooterIOM;
    private final ShooterIO shooterIOR;
    private final ShooterIO feederIO;
    private final ShooterIO indexIO;

    private final LoggedNetworkNumber loggedRadPerSec = new LoggedNetworkNumber("Shooter/Tuning/Setpoint", ShooterConstants.kP);

    private final ShooterIOInputsAutoLogged[] shooterInputs = new ShooterIOInputsAutoLogged[5];

    public Shooter(ShooterIO shooterIOL, ShooterIO shooterIOM, ShooterIO shooterIOR, ShooterIO feederIO, ShooterIO indexIO) {
        this.shooterIOL = shooterIOL;
        this.shooterIOM = shooterIOM;
        this.shooterIOR = shooterIOR;
        this.feederIO = feederIO;
        this.indexIO = indexIO;
    }

    private void shootWithDistance(double distanceMeters) {
        double shooterVelocityRadPerSec = HubShootLUT.getFlywheelSpeedAtDistance(distanceMeters);
        double feederVelocityRadPerSec = shooterVelocityRadPerSec * ShooterConstants.feederMotorMult;

        shooterIOL.setVelocityClosedLoop(shooterVelocityRadPerSec);
        shooterIOM.setVelocityClosedLoop(shooterVelocityRadPerSec);
        shooterIOR.setVelocityClosedLoop(shooterVelocityRadPerSec);
        feederIO.setVelocityClosedLoop(feederVelocityRadPerSec);
        indexIO.setVelocityClosedLoop(feederVelocityRadPerSec);
    }

    public Command runStopShooter() {
        return run(() -> {
            shooterIOL.stop();
            shooterIOM.stop();
            shooterIOR.stop();
            feederIO.stop();
            indexIO.stop();
        });
    }

    public Command runShootMaxSpeed() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            feederIO.setVelocityClosedLoop(ShooterConstants.feederMotorMaxSpeed);
            indexIO.setVelocityClosedLoop(ShooterConstants.feederMotorMaxSpeed);
        });
    }

    public Command runShooterIdle() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            feederIO.stop();
            indexIO.stop();
        });
    }

    public Command runShootAtHub(Supplier<Pose2d> poseSupplier) {
        return run(() -> {
            Pose2d robotPose = poseSupplier.get();
            Pose2d hubPose = FieldConstants.getHubCenter();

            double hubDistance = robotPose.getTranslation().getDistance(hubPose.getTranslation());

            shootWithDistance(hubDistance);
        });
    }

    public Command runShootOneShooter() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(loggedRadPerSec.get());
        });
    }

    @Override
    public void periodic() {
        shooterIOL.periodic();
        shooterIOM.periodic();
        shooterIOR.periodic();
        feederIO.periodic();
        indexIO.periodic();

        shooterIOL.updateInputs(shooterInputs[0]);
        shooterIOM.updateInputs(shooterInputs[1]);
        shooterIOR.updateInputs(shooterInputs[2]);
        feederIO.updateInputs(shooterInputs[3]);
        indexIO.updateInputs(shooterInputs[4]);

        Logger.processInputs("Shooter/RightShooter", shooterInputs[0]);
        Logger.processInputs("Shooter/MiddleShooter", shooterInputs[1]);
        Logger.processInputs("Shooter/LeftShooter", shooterInputs[2]);
        Logger.processInputs("Shooter/Feeder", shooterInputs[3]);
        Logger.processInputs("Shooter/Indexer", shooterInputs[4]);
    }
}
