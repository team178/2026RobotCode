package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Constants {
    public static enum RobotMode {
		/** Running on a real robot. */
		REAL,
	
		/** Running a physics simulator. */
		SIM,
	
		/** Replaying from a log file. */
		REPLAY;
	}

    public static final RobotMode simMode = RobotMode.SIM;
	public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : simMode;

    public static boolean isRed() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static enum FieldType {
		ANDYMARK, WELDED
	}

    public static final SendableChooser<FieldType> kFieldType = new SendableChooser<>();

    static {
        kFieldType.setDefaultOption("AndyMark Field", FieldType.ANDYMARK);
        kFieldType.setDefaultOption("Welded Field", FieldType.WELDED);
    }

    public static class SwerveConstants {
        public static final int[] turnCANIDs = { 1, 2, 3, 4 };
        public static final int[] driveCANIDs = { 5, 6, 7, 8 };
        public static final int[] canCoderCANIDs = { 9, 10, 11, 12 };
        public static final int pigeonCANID = 13;

        public static final double kWheelDistanceMetersX = Units.inchesToMeters(20); // forward/back
        public static final double kWheelDistanceMetersY = Units.inchesToMeters(20); // left/right

        public static final double kSlowedMult = 0.12;
        
        public static final double kMaxWheelSpeed = 20; // m/2
        public static final double kMagVelLimit = 5; // m/s -- trusting andy that 5 is physical limit, maybe requires testing
        public static final double kRotVelLimit = 18; // rad/s

        public static final double toXDelaySeconds = 1;

        // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
        /**
         * Get robot's initial pose based on field type and alliance
         * @return
         */
        public static Pose2d getInitialPose() {
            if (isRed()) {
                if (kFieldType.getSelected().equals(FieldType.WELDED)) {
                    return new Pose2d(
                        Units.inchesToMeters(651.22),
                        Units.inchesToMeters(317.69),
                        Rotation2d.k180deg
                    );
                } else {
                    return new Pose2d(
                        Units.inchesToMeters(650.12),
                        Units.inchesToMeters(316.64),
                        Rotation2d.k180deg
                    );
                }
            } else {
                return new Pose2d();
            }
        }

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

        public static final double kDriveMotorGearRatio = 6.12; // number of encoder rotations per wheel rotation
        public static final double kSwerveWheelDiameter = 4; // inches

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
            // turnConfig.closedLoop
            //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            //     .pid(turnP, turnI, turnD)
            //     .positionWrappingEnabled(true)
            //     .positionWrappingInputRange(0, 2 * Math.PI)
            //     .outputRange(-1,1);
            // turnConfig.absoluteEncoder
            //     .positionConversionFactor(2 * Math.PI)
            //     .velocityConversionFactor(2 * Math.PI);

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

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kAuxControllerPort = 1;
    }
}
