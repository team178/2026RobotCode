package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double positionRad = 0;
        public double velocityRadPerSec = 0;
        
        public double appliedVolts = 0;
        public double currentAmps = 0;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setClosedLoop(double velocityRadPerSec);

    public void setOpenLoop(double voltage);
}
