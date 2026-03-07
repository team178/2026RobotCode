package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSpark implements ShooterIO {

    public final SparkMax motor;
    public final RelativeEncoder motorEncoder;
    public final SparkClosedLoopController motorController;

    public ShooterIOSpark(int CANID) {
        motor = new SparkMax(CANID, MotorType.kBrushless);
        motor.setCANTimeout(0);

        motorController = motor.getClosedLoopController();
        motorEncoder = motor.getEncoder();

        SparkMaxConfig motorConfig = ShooterConstants.feederConfig;
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override 
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        // feedforward should already be accounted for
        motorController.setSetpoint(
            velocityRadPerSec,
            ControlType.kVelocity
        );
    }

    @Override 
    public void setOpenLoop(double voltage) {
        motor.setVoltage(voltage); 
    }

    @Override 
    public void stop(){
        motor.stopMotor(); 
    }
    
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocityRadPerSec = motorEncoder.getVelocity();

        inputs.appliedVolts = motor.getBusVoltage();
        inputs.supplyCurrentAmps = motor.getOutputCurrent();
    }
}
