package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.Constants.IntakeConstants;

public class RollerIOSpark implements RollerIO {
    private final SparkMax intakeMotor;
    private final SparkClosedLoopController intakeController;
    private final SparkAbsoluteEncoder intakeEncoder;

    public RollerIOSpark() {
        intakeMotor = new SparkMax(IntakeConstants.kWristCANID, MotorType.kBrushless);
        intakeController = intakeMotor.getClosedLoopController();
        intakeEncoder = intakeMotor.getAbsoluteEncoder();

        intakeMotor.configure(IntakeConstants.rollerSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.positionRad = intakeEncoder.getPosition();
        inputs.velocityRadPerSec = intakeEncoder.getVelocity();

        inputs.appliedVolts = intakeMotor.getBusVoltage();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
    }

    @Override
    public void setClosedLoop(double velocityRadPerSec) {
        intakeController.setSetpoint(velocityRadPerSec, ControlType.kVelocity);
    }

    @Override
    public void setOpenLoop(double voltage) {
        intakeMotor.setVoltage(voltage);
    }
    
}
