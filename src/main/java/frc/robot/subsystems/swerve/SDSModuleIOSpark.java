package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.Constants.SwerveConstants;
import frc.robot.subsystems.Constants.SwerveModuleConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SDSModuleIOSpark implements SDSModuleIO {
    private final SparkMax turnMotor;
    private final SparkMax driveMotor;

    private final SparkAbsoluteEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;

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

        turnEncoder = turnMotor.getAbsoluteEncoder();
        driveEncoder = turnMotor.getEncoder();

        turnController = turnMotor.getClosedLoopController();
        driveController = driveMotor.getClosedLoopController();

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void updateInputs(SDSModuleIOInputs inputs) {
        inputs.turnPosition = new Rotation2d(/*Put ID constant here*/);
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
    }

    public void setTurnPosition(Rotation2d position) {

    }

    public void setDriveVelocityRadPerSec(double velocityRadPerSec) {

    }

    public void setTurnOpenLoop(double output) {

    }

    public void setDriveOpenLoop(double output) {
        
    }

    public void updateControlConstants() {
        
    }
}