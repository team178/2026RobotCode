package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSpark implements ClimbIO {
    private final SparkMax climbMotor;
    private final RelativeEncoder climbEncoder;
    private final BangBangController climbController;

    public ClimbIOSpark() {
        climbMotor = new SparkMax(ClimbConstants.climberMotorCANID, MotorType.kBrushless);
        climbEncoder = climbMotor.getEncoder();
        climbController = new BangBangController();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.positionRotations = climbEncoder.getPosition();
        inputs.velocityRotPerSec = climbEncoder.getVelocity();

        inputs.appliedVolts = climbMotor.getAppliedOutput();
        inputs.currentAmps = climbMotor.getOutputCurrent();
    }

    @Override
    public void setClosedLoop(double rotations) {
        double voltage = 12 * climbController.calculate(climbEncoder.getPosition(), rotations);
        climbMotor.setVoltage(voltage);
    }

    @Override
    public void setOpenLoop(double voltage) {
        climbMotor.setVoltage(voltage);
    }
}
