package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// sim based on the advantagekit 2026 swerve example
public class SDSModuleIOSim implements SDSModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // TODO get actual robot reduction
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // rotor rotations -> wheel radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // RPM -> wheel rad/sec

    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    public static final double turnMotorReduction = 9424.0 / 203.0; // TODO get actual robot reduction
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    public static final double turnEncoderPositionFactor = 2 * Math.PI; // rot -> rad
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> rad/sec

    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // rad
    public static final double turnPIDMaxInput = 2 * Math.PI; // rad

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
    private PIDController turnController = new PIDController(turnSimP, 0, turnSimD);

    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SDSModuleIOSim() {
        driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox
        );
        turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox
        );

        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SDSModuleIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }

        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.turnConnected = true;
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocityRadPerSec(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
