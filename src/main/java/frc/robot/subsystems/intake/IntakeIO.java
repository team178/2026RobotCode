package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setClosedLoop(double voltage);

    public void setOpenLoop(double voltage);
}
