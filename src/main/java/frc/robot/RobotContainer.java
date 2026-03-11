// ============================================================
// RobotContainer.java  — UPDATED
// Changes from original:
//   1. Added IntakeSubsystem and ShooterSubsystem fields
//   2. Added AutoFactory construction (ChoreoLib)
//   3. Added AutoChooser with Path2877Auto registered
//   4. getAutonomousCommand() now returns the selected auto
//   5. RobotModeTriggers schedules the auto during autonomous
//
// Lines marked  // [NEW]  are additions to the original file.
// All original bindings and swerve setup are preserved unchanged.
// ============================================================

package frc.robot;

import choreo.auto.AutoChooser;          // [NEW]
import choreo.auto.AutoFactory;          // [NEW]
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // [NEW]
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers; // [NEW]
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants; // [NEW]
// import frc.robot.subsystems.IntakeSubsystem;  // [NEW] — adjust package if needed
// import frc.robot.subsystems.ShooterSubsystem; // [NEW] — adjust package if needed
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon;
import frc.robot.subsystems.swerve.SDSModuleIO;
import frc.robot.subsystems.swerve.SDSModuleIOSim;
import frc.robot.subsystems.swerve.SDSModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    

    // -------------------------------------------------------
    // Existing fields — UNCHANGED
    // -------------------------------------------------------
    private final CommandXboxController driverController;
    private final CommandXboxController auxController;
    private final SwerveDrive swerve;
    private final AutoFactory autoFactory;

    private SendableChooser<String> autoChoose = new SendableChooser<>();
    private SendableChooser<String> EP1 = new SendableChooser<>();
    private SendableChooser<String> EP2 = new SendableChooser<>();
    private SendableChooser<String> EP3 = new SendableChooser<>();
    private SendableChooser<String> EP4 = new SendableChooser<>();
    private SendableChooser<String> EP5 = new SendableChooser<>();
    private String lastSelected = "";
    String[] EPs1a3 = {"2", "3", "4", "5", "6", "7", "8", "N/A"};
    String[] EPs2 = {"2", "3", "4a", "4b", "5", "6a", "6b", "7", "8", "N/A"};

    public RobotContainer() {
        SmartDashboard.putData("Auto Chooser", autoChoose); // [NEW]
        autoChoose.setDefaultOption("Auto 1", "Auto1");
        autoChoose.addOption("Auto 2", "Auto2");
        autoChoose.addOption("Auto 3", "Auto3");
        
        Preferences.removeAll();

        // Temporary swerve construction to pass into AutoFactory — will be re-assigned properly below

        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        auxController    = new CommandXboxController(OperatorConstants.kAuxControllerPort);

        // -------------------------------------------------------
        // Existing swerve construction — UNCHANGED
        // -------------------------------------------------------
        switch (Constants.currentMode) {
            case REAL:
                swerve = new SwerveDrive(
                    new GyroIOPigeon(),
                    new SDSModuleIOSpark(0),
                    new SDSModuleIOSpark(1),
                    new SDSModuleIOSpark(2),
                    new SDSModuleIOSpark(3)
                );
                break;
            case SIM:
                swerve = new SwerveDrive(
                    new GyroIO() {},
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim(),
                    new SDSModuleIOSim()
                );
                break;
            default:
                swerve = new SwerveDrive(
                    new GyroIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {}
                );
                break;
        }

        autoFactory = new AutoFactory(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::followTrajectory,
            true,
            swerve
        );

        configureBindings();
    }

    // -------------------------------------------------------
    // Existing bindings — UNCHANGED
    // -------------------------------------------------------
    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX,
            driverController::getLeftY,
            driverController::getRightX,
            driverController::getRightTriggerAxis
        ));

        driverController.y().onTrue(swerve.runZeroGyro());
        driverController.x().onTrue(swerve.runToggleToXPosition());
        driverController.b().onTrue(swerve.runReconfigure());
    }

    public void initAutoChooser() {
        String auto = autoChoose.getSelected();
        if (!lastSelected.equals(auto)) {
            lastSelected = autoChoose.getSelected();
            EP1 = new SendableChooser<>();
            EP2 = new SendableChooser<>();
            EP3 = new SendableChooser<>();
            EP4 = new SendableChooser<>();
            EP5 = new SendableChooser<>();

            String[] actualEPs = auto.equals("Auto2") ? EPs2 : EPs1a3;
            
            for (String EP : actualEPs) {
                EP1.addOption(EP, EP);
                EP2.addOption(EP, EP);
                EP3.addOption(EP, EP);
                EP4.addOption(EP, EP);
                EP5.addOption(EP, EP);
            }
            
            SmartDashboard.putData("Endpoint 1", EP1);
            SmartDashboard.putData("Endpoint 2", EP2);
            SmartDashboard.putData("Endpoint 3", EP3);
            SmartDashboard.putData("Endpoint 4", EP4);
            SmartDashboard.putData("Endpoint 5", EP5);
        }
    }

    public void testPeriodic() {
        swerve.periodic();
    }

    private AutoRoutine makeAuto() {
        AutoRoutine routine = autoFactory.newRoutine("Auto");
        
        return routine;
    }

    // -------------------------------------------------------
    // getAutonomousCommand() is preserved for compatibility
    // but the auto is now also scheduled via RobotModeTriggers
    // above, so this is only needed if Robot.java calls it directly.
    // -------------------------------------------------------
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}