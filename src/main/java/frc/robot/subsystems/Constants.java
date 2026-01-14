package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class SwerveConstants {
        public static final int[] turnCANIDs = { 1, 2, 3, 4 };
        public static final int[] driveCANIDs = { 5, 6, 7, 8 };

        public static final int pigeonCANID = 9;
    }

    public static class SwerveModuleConstants {
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();

        public static final Rotation2d[] zeroRotations = {
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d()
        };
    }
}
