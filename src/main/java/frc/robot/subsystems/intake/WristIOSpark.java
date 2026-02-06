package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.subsystems.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOSpark implements WristIO {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController wristController;
    private final SparkAbsoluteEncoder wristEncoder;

    public WristIOSpark() {
        wristMotor = new SparkMax(IntakeConstants.kWristCANID, MotorType.kBrushless);
        wristController = wristMotor.getClosedLoopController();
        wristEncoder = wristMotor.getAbsoluteEncoder();

        wristMotor.configure(IntakeConstants.wristSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.position = new Rotation2d(wristEncoder.getPosition());
        inputs.velocityRadPerSec = wristEncoder.getVelocity();

        inputs.appliedVolts = wristMotor.getBusVoltage();
        inputs.currentAmps = wristMotor.getOutputCurrent();
    }

    @Override
    public void setSetpoint(double angle) {
        wristController.setSetpoint(angle, ControlType.kPosition);
    }

    @Override
    public void setOpenLoop(double voltage) {
        wristMotor.setVoltage(voltage);
    }
}
