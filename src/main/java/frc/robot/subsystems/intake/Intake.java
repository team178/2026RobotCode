package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static enum IntakePose {
        RETRACTED(0),
        EXTENDED(1);

        IntakePose(double setpoint) {

        }
    }

    public final IntakeIO intakeIO;
    public final WristIO wristIO;

    public Intake(IntakeIO intakeIO, WristIO wristIO) {
        this.intakeIO = intakeIO;
        this.wristIO = wristIO;
    }

    public void setPose(IntakePose pose) {
        
    }
}
