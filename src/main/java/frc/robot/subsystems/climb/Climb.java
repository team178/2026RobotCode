package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants.ClimbPose;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    public ClimbIO climbIO;
    public ClimbIOInputsAutoLogged inputs;

    public Climb(ClimbIO climbIO) {
        this.climbIO = climbIO;
    }

    public Command runExtend() {
        return run(() -> {
            climbIO.setClosedLoop(ClimbPose.EXTENDED.getSetpoint());
        });
    }

    public Command runRetract() {
        return run(() -> {
            climbIO.setClosedLoop(ClimbPose.RETRACTED.getSetpoint());
        });
    }

    public Command stopClimb() {
        return run(() -> {
            climbIO.setOpenLoop(0);
        });
    }

    @Override
    public void periodic() {
        climbIO.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
