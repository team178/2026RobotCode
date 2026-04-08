package frc.robot.subsystems;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autos.AutoBrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoCreator extends SubsystemBase {
    private AutoRoutine auto;
    private Command autoCommand;
    private AutoBrain autoBrain;
    private LoggedNetworkNumber trigger = new LoggedNetworkNumber("saveAuto", 0);
    private double lastSaveValue = -1;

    public AutoCreator(SwerveDrive swerve, Shooter shooter, Intake intake) {
        autoBrain = new AutoBrain(swerve, shooter, intake);
    }

    public Command returnAuto() {
        return autoCommand;
    }

    @Override
    public void periodic() {
        double thisSaveValue = trigger.getAsDouble();
        if(thisSaveValue != lastSaveValue) {
            auto = autoBrain.buildAuto();
            autoCommand = auto.cmd();
            lastSaveValue = thisSaveValue;
        }
    }
}
