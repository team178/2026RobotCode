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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterIOSparkFeeder implements ShooterIO {

    public final SparkMax feederMotor;
    public final RelativeEncoder feederEncoder;
    public final SparkClosedLoopController feederController;
    public final SparkMaxConfig motorConfig;

    private final LoggedNetworkNumber loggedKP = new LoggedNetworkNumber("Shooter/Feeder/Tuning/kP", ShooterConstants.kP);
    private final LoggedNetworkNumber loggedKD = new LoggedNetworkNumber("Shooter/Feeder/Tuning/kD", ShooterConstants.kD);
    private final LoggedNetworkNumber loggedKS = new LoggedNetworkNumber("Shooter/Feeder/Tuning/kS", ShooterConstants.kS);
    private final LoggedNetworkNumber loggedKV = new LoggedNetworkNumber("Shooter/Feeder/Tuning/kV", ShooterConstants.kV);

    private double lastKp = 0.0;
    private double lastKd = 0.0;
    private double lastKs = 0.0;
    private double lastKv = 0.0;

    public ShooterIOSparkFeeder(int CANID) {
        feederMotor = new SparkMax(CANID, MotorType.kBrushless);
        feederMotor.setCANTimeout(0);

        feederController = feederMotor.getClosedLoopController();
        feederEncoder = feederMotor.getEncoder();

        motorConfig = ShooterConstants.feederConfig;
        feederMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double currentKp = loggedKP.get();
        double currentKd = loggedKD.get();
        double currentKs = loggedKS.get();
        double currentKv = loggedKV.get();

        if (currentKp != lastKp || currentKd != lastKd || currentKs != lastKs || currentKv != lastKv) {
            motorConfig.closedLoop
                .p(currentKp)
                .d(currentKd);
            motorConfig.closedLoop.feedForward
                .kV(currentKv)
                .kS(currentKs);

            feederMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastKp = currentKp;
            lastKd = currentKd;
            lastKs = currentKs;
            lastKv = currentKv;

            System.out.println("SparkMax Constants Updated!");
        }
    }

    @Override 
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        Logger.recordOutput("Shooter/Feeder/Setpoint", velocityRadPerSec);

        // feedforward should already be accounted for
        feederController.setSetpoint(
            velocityRadPerSec,
            ControlType.kVelocity
        );
    }

    @Override 
    public void setOpenLoop(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    @Override 
    public void stop(){
        feederMotor.stopMotor();
    }
    
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocityRadPerSec = feederEncoder.getVelocity();

        inputs.appliedVolts = feederMotor.getBusVoltage();
        inputs.supplyCurrentAmps = feederMotor.getOutputCurrent();
    }
}
