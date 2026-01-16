package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class SwerveConstants {
        public static final int[] turnCANIDs = { 1, 2, 3, 4 };
        public static final int[] driveCANIDs = { 5, 6, 7, 8 };

        public static final int pigeonCANID = 9;
		public static double kWheelRadiusMeters = 9;
        //legacy
    }

    public static class SwerveModuleConstants {
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();

        public static final double turnP = 0;
        public static final double turnI = 0;
        public static final double turnD = 0;

        public static final double driveP = 0;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double driveFF = 0;

        public static final double DriveMotorGearRatio = 6.12; //number of encoder rotations per wheel rotation
        public static final double SwerveWheelDiameter = 4; //inches

        public static final double kMaxSpeed = 0;

        public static final Rotation2d[] zeroRotations = {
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d()
        };

        static {
            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12);
            turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(turnP, turnI, turnD)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1,1);
            turnConfig.absoluteEncoder
                .positionConversionFactor(2 * Math.PI)
                .velocityConversionFactor(2 * Math.PI);

            driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
                .closedLoopRampRate(0.01);
            driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(driveP, driveI, driveD)
                .outputRange(-1,1)
                .feedForward.kV(driveFF);
            driveConfig.absoluteEncoder
                .positionConversionFactor(2 * Math.PI)
                .velocityConversionFactor(2 * Math.PI);
        }
    }
}
