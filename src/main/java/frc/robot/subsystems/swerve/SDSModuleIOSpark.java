package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Constants.SwerveConstants;
import frc.robot.subsystems.Constants.SwerveModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController; 
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SDSModuleIOSpark implements SDSModuleIO {
    private final SparkMax turnMotor;
    private final SparkMax driveMotor;

    private final RelativeEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;

    private final CANcoder turnCANCoder;

    private final SparkClosedLoopController turnController;
    private final SparkClosedLoopController driveController;

    private final int index;

    public SDSModuleIOSpark(int index) {
        this.index = index;

        SparkMaxConfig turnConfig = SwerveModuleConstants.turnConfig;
        SparkMaxConfig driveConfig = SwerveModuleConstants.driveConfig;

        turnMotor = new SparkMax(SwerveConstants.turnCANIDs[index], MotorType.kBrushless);
        driveMotor = new SparkMax(SwerveConstants.driveCANIDs[index], MotorType.kBrushless);

        turnMotor.setCANTimeout(0);
        driveMotor.setCANTimeout(0);

        turnEncoder = turnMotor.getEncoder();
        driveEncoder = driveMotor.getEncoder();

        turnController = turnMotor.getClosedLoopController();
        driveController = driveMotor.getClosedLoopController();

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // TODO research to figure out if you can save StatusSignal as a variable instead of doing .getAbsolutePosition every time on the CANCoders. what is recommended?
        turnCANCoder = new CANcoder(SwerveConstants.canCoderCANIDs[index]);
        turnEncoder.setPosition(turnCANCoder.getAbsolutePosition().getValueAsDouble() * (2 * Math.PI)); // TODO test to make sure this sets the motor to the correct position
    }
    
    public void updateInputs(SDSModuleIOInputs inputs) {
        inputs.turnConnected = turnMotor.getFirmwareVersion() != 0;
        inputs.turnPosition = new Rotation2d(
            turnEncoder.getPosition()
            // - SwerveModuleConstants.zeroRotations[index].getRadians()
            // turnCANCoder.getAbsolutePosition().getValueAsDouble() * (2 * Math.PI) - SwerveModuleConstants.zeroRotations[index].getRadians()
        );
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.canCoderPosition = turnCANCoder.getAbsolutePosition().getValueAsDouble();
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

        inputs.driveConnected = driveMotor.getFirmwareVersion() != 0;
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveVelocityWheelMetersPerSec = inputs.driveVelocityRadPerSec * SwerveModuleConstants.kSwerveWheelDiameter / 2.0;
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
    }

    // TODO make sure that zero rotation is applied correctly, ensure logic is correct
    public void setTurnPosition(Rotation2d position) {
        turnController.setSetpoint(position.getRadians() + SwerveModuleConstants.zeroRotations[index].getRadians(), ControlType.kPosition);
        // turnController.setSetpoint(0, ControlType.kPosition);
    }

    public void setDriveVelocityRadPerSec(double velocityRadPerSec) {
        driveController.setSetpoint(velocityRadPerSec, ControlType.kVelocity);
        // driveController.setSetpoint(2 * Math.PI, ControlType.kVelocity);
    }

    public void setTurnOpenLoop(double output) {
        turnMotor.setVoltage(output);
    }

    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }

    // public void updateControlConstants() {
        
    // }
}
