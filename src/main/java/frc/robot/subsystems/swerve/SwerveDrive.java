package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.GyroIO.GyroIOInputs;

public class SwerveDrive {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroIOInputs;
    
    private final SDSSwerveModule[] modules;

    public SwerveDrive(
        GyroIO gyroIO,
        SDSModuleIO flModuleIO,
        SDSModuleIO frModuleIO,
        SDSModuleIO blModuleIO,
        SDSModuleIO brModuleIO
    ) {
        this.gyroIO = gyroIO;
        this.gyroIOInputs = new GyroIOInputsAutoLogged();

        modules = new SDSSwerveModule[] {
            new SDSSwerveModule(0, flModuleIO),
            new SDSSwerveModule(1, frModuleIO),
            new SDSSwerveModule(2, blModuleIO),
            new SDSSwerveModule(3, brModuleIO)
        };
    }
}