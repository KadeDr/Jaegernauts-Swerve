package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ToggleIntake.IntakeModule;
import frc.robot.subsystems.ToggleIntake.IntakeModuleState;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeModule m_intake = new IntakeModule(12);
    private final double m_waitTime = 0.25;

    public Command setDesiredStateCommand(Rotation2d angle) {
        return runOnce(() -> {
            m_intake.SetState(new IntakeModuleState(angle));
        });
    }

    public SequentialCommandGroup raiseIntake() {
        return new SequentialCommandGroup(
                setDesiredStateCommand(new Rotation2d().fromRadians(3.1)),
                Commands.waitUntil(() -> m_intake.atSetpoint()),
                (setDesiredStateCommand(new Rotation2d().fromRadians(6))));
    }

    public SequentialCommandGroup lowerIntake() {
        return new SequentialCommandGroup(
                setDesiredStateCommand(new Rotation2d().fromRadians(3.2)),
                Commands.waitUntil(() -> m_intake.atSetpoint()),
                setDesiredStateCommand(new Rotation2d().fromRadians(3.1)),
                Commands.waitUntil(() -> m_intake.atSetpoint()),
                (setDesiredStateCommand(new Rotation2d().fromRadians(0))));
    }

    public IntakeSubsystem() {
        SmartDashboard.putData("Raise Intake", raiseIntake());
        SmartDashboard.putData("Lower Intake", lowerIntake());
    }

    public boolean atSetpoint() {
        return m_intake.atSetpoint();
    }
    
    public IntakeModuleState getPosition() {
        return m_intake.getState();
    }
}
