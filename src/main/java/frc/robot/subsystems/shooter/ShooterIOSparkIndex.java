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

public class ShooterIOSparkIndex implements ShooterIO {

    public final SparkMax indexMotor;
    public final RelativeEncoder indexEncoder;
    public final SparkClosedLoopController indexController;
    public final SparkMaxConfig motorConfig;

    public final LoggedTunableControlConstants controlConstants = new LoggedTunableControlConstants("Shooter/Index");

    public ShooterIOSparkIndex(int CANID) {
        indexMotor = new SparkMax(CANID, MotorType.kBrushless);
        indexMotor.setCANTimeout(0);

        indexController = indexMotor.getClosedLoopController();
        indexEncoder = indexMotor.getEncoder();

        motorConfig = ShooterConstants.indexConfig;
        indexMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controlConstants
            .setP(ShooterConstants.indexkP)
            .setD(ShooterConstants.indexkD)
            .setS(ShooterConstants.indexkS)
            .setV(ShooterConstants.indexkV);
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

            indexMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            System.out.println("Control Constants Updated!");
        });
    }

    @Override 
    public void setVelocityClosedLoop(double velocityRadPerSec) {
        Logger.recordOutput("Shooter/Index/Setpoint", velocityRadPerSec);

        // feedforward should already be accounted for
        indexController.setSetpoint(
            velocityRadPerSec,
            ControlType.kVelocity
        );
    }

    @Override 
    public void setOpenLoop(double voltage) {
        indexMotor.setVoltage(voltage);
    }

    @Override 
    public void stop(){
        indexMotor.stopMotor();
    }
    
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocityRadPerSec = indexEncoder.getVelocity();

        inputs.appliedVolts = indexMotor.getBusVoltage() * indexMotor.getAppliedOutput();
        inputs.supplyCurrentAmps = indexMotor.getOutputCurrent();
    }
}
