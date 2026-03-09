package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableControlConstants;

import org.littletonrobotics.junction.Logger;

public class ShooterIOTalonFlywheel implements ShooterIO {
    public final TalonFX motor;
    public final int CANID;

    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0);

    public final LoggedTunableControlConstants controlConstants = new LoggedTunableControlConstants("Shooter/Flywheel");

    public ShooterIOTalonFlywheel(int CANID) {
        motor = new TalonFX(CANID);
        this.CANID = CANID;

        motor.getConfigurator().apply(ShooterConstants.talonFlywheelConfigs);

        controlConstants
            .setP(ShooterConstants.kP)
            .setD(ShooterConstants.kD)
            .setS(ShooterConstants.kS)
            .setV(ShooterConstants.kV);
    }

    @Override
    public void periodic() {
        controlConstants.setCallback((double kP, double kI, double kD, double kS, double kV, double kCos) -> {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = kP;
            slot0.kD = kD;
            slot0.kS = kS;
            slot0.kV = kV;

            motor.getConfigurator().apply(slot0, 0);

            System.out.println("Control Constants Updated!");
        });
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
