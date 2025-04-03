package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleIntake.IntakeModule;
import frc.robot.subsystems.ToggleIntake.IntakeModuleState;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeModule m_intake = new IntakeModule(13);
    private final double m_waitTime = 0.25;

    public Command setDesiredStateCommand(Rotation2d angle) {
        return runOnce(() -> {
            m_intake.SetState(new IntakeModuleState(angle));
        });
    }

    public IntakeSubsystem() {
        SmartDashboard.putData("Raise Intake",
                setDesiredStateCommand(new Rotation2d().fromRadians(3.1)).andThen(Commands.waitSeconds(m_waitTime))
                        .andThen(setDesiredStateCommand(new Rotation2d().fromRadians(5))));
        SmartDashboard.putData("Lower Intake",
                setDesiredStateCommand(new Rotation2d().fromRadians(3.2)).andThen(Commands.waitSeconds(m_waitTime))
                        .andThen(setDesiredStateCommand(new Rotation2d().fromRadians(3.1)))
                        .andThen(Commands.waitSeconds(m_waitTime))
                        .andThen(setDesiredStateCommand(new Rotation2d().fromRadians(0))));
        setDesiredStateCommand(new Rotation2d(0, 0));
    }
}
