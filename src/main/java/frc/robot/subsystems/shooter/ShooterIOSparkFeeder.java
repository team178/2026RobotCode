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
import frc.robot.util.LoggedTunableControlConstants;

import org.littletonrobotics.junction.Logger;

public class ShooterIOSparkFeeder implements ShooterIO {

    public final SparkMax feederMotor;
    public final RelativeEncoder feederEncoder;
    public final SparkClosedLoopController feederController;
    public final SparkMaxConfig motorConfig;

    public final LoggedTunableControlConstants controlConstants = new LoggedTunableControlConstants("Shooter/Feeder");

    public ShooterIOSparkFeeder(int CANID) {
        feederMotor = new SparkMax(CANID, MotorType.kBrushless);
        feederMotor.setCANTimeout(0);

        feederController = feederMotor.getClosedLoopController();
        feederEncoder = feederMotor.getEncoder();

        motorConfig = ShooterConstants.feederConfig;
        feederMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controlConstants
            .setP(ShooterConstants.kP)
            .setD(ShooterConstants.kD)
            .setS(ShooterConstants.kS)
            .setV(ShooterConstants.kV);
    }

    @Override
    public void periodic() {
        controlConstants.setCallback((double kP, double kI, double kD, double kS, double kV, double kCos) -> {
            motorConfig.closedLoop
                .p(kP)
                .d(kD);
            motorConfig.closedLoop.feedForward
                .kV(kV)
                .kS(kS);

            feederMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            System.out.println("Control Constants Updated!");
        });
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

        inputs.appliedVolts = feederMotor.getBusVoltage() * feederMotor.getAppliedOutput();
        inputs.supplyCurrentAmps = feederMotor.getOutputCurrent();
    }
}
