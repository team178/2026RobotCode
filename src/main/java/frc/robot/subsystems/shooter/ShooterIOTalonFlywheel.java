package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterIOTalonFlywheel implements ShooterIO {
    public final TalonFX motor;
    public final int CANID;

    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0);

    private final LoggedNetworkNumber loggedKP = new LoggedNetworkNumber("Shooter/Flywheel/Tuning/kP", ShooterConstants.kP);
    private final LoggedNetworkNumber loggedKD = new LoggedNetworkNumber("Shooter/Flywheel/Tuning/kD", ShooterConstants.kD);
    private final LoggedNetworkNumber loggedKS = new LoggedNetworkNumber("Shooter/Flywheel/Tuning/kS", ShooterConstants.kS);
    private final LoggedNetworkNumber loggedKV = new LoggedNetworkNumber("Shooter/Flywheel/Tuning/kV", ShooterConstants.kV);

    private double lastKp = 0.0;
    private double lastKd = 0.0;
    private double lastKs = 0.0;
    private double lastKv = 0.0;

    public ShooterIOTalonFlywheel(int CANID) {
        motor = new TalonFX(CANID);
        this.CANID = CANID;
        
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs(); 
        outputConfigs.NeutralMode = NeutralModeValue.Coast; // IMPORTANT
        motor.getConfigurator().apply(outputConfigs);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        var slot0 = talonConfigs.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kD = ShooterConstants.kD;
        slot0.kS = ShooterConstants.kS;
        slot0.kV = ShooterConstants.kV;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.mmCruise;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.mmAcceleration;
        motionMagicConfigs.MotionMagicJerk = ShooterConstants.mmJerk;

        motor.getConfigurator().apply(talonConfigs);
    }

    @Override
    public void periodic() {
        double currentKp = loggedKP.get();
        double currentKd = loggedKD.get();
        double currentKs = loggedKS.get();
        double currentKv = loggedKV.get();

        if (currentKp != lastKp || currentKd != lastKd || currentKs != lastKs || currentKv != lastKv) {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = currentKp;
            slot0.kD = currentKd;
            slot0.kS = currentKs;
            slot0.kV = currentKv;

            motor.getConfigurator().apply(slot0);

            lastKp = currentKp;
            lastKd = currentKd;
            lastKs = currentKs;
            lastKv = currentKv;

            System.out.println("TalonFX Constants Updated!");
        }
    }

    @Override
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        Logger.recordOutput("Shooter/Flywheel/" + CANID + "/Setpoint", velocityRadPerSec);

        motor.setControl(
            velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec))
        );
    }

    @Override 
    public void setOpenLoop(double voltage) {
        motor.setVoltage(voltage); 
    }

    @Override 
    public void stop(){
        motor.setVoltage(0);
    }

    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    }
}
