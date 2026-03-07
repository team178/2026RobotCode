package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.IntakeConstants;
import frc.robot.subsystems.Constants.IntakeConstants.IntakeWristPose;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Intake extends SubsystemBase {
    public final RollerIO rollerIO;
    public final WristIO wristIO;

    LoggedDashboardChooser<IntakeWristPose> poseChooser = new LoggedDashboardChooser("Intake/Wrist/Pose");

    public Intake(RollerIO rollerIO, WristIO wristIO) {
        this.rollerIO = rollerIO;
        this.wristIO = wristIO;

        poseChooser.addDefaultOption("Retracted", IntakeWristPose.RETRACTED);
        poseChooser.addOption("Extended", IntakeWristPose.EXTENDED);
    }

    public void setPose(IntakeWristPose pose) {
        wristIO.setSetpoint(pose.getSetpoint());
    }

    public void intake() {
        rollerIO.setClosedLoop(IntakeConstants.intakeMaxSpeed);
    }

    public void stopIntake() {
        rollerIO.setOpenLoop(0);
    }

    public void stopWrist() {
        wristIO.setOpenLoop(0);
    }

    public Command runIntake() {
        return run(() -> {
            intake();
        });
    }

    public Command runJustWrist() {
        return run(() -> {
            setPose(poseChooser.get());
        });
    }

    public Command runStop() {
        return run(() -> {
            stopIntake();
            stopWrist();
        });
    }

    @Override
    public void periodic() {
        rollerIO.periodic();
        wristIO.periodic();
    }
}
