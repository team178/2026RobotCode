package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Constants.IntakeConstants;

public class WristIOTalon implements WristIO {
    private final TalonFX wristMotor;
    private final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    public WristIOTalon() {
        wristMotor = new TalonFX(IntakeConstants.kWristCANID);

        wristMotor.getConfigurator().apply(IntakeConstants.wristTalonConfig);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.position = new Rotation2d(wristMotor.getPosition().getValueAsDouble());
        inputs.velocityRadPerSec = wristMotor.getVelocity().getValueAsDouble();

        inputs.appliedVolts = wristMotor.getSupplyVoltage().getValueAsDouble();
        inputs.currentAmps = wristMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setSetpoint(double angle) {
        wristMotor.setControl(voltageRequest.withPosition(angle));
    }

    @Override
    public void setOpenLoop(double voltage) {
        wristMotor.setVoltage(voltage);
    }
}
