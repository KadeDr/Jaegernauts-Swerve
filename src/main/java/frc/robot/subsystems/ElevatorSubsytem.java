package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorModule;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsytem extends SubsystemBase {
    private final ElevatorModule m_elevator = new ElevatorModule(ElevatorConstants.kLeftCANId, ElevatorConstants.kRightCANId, ElevatorConstants.kIntakeCANId);

    public void MoveElevator(double speed) {
        m_elevator.SetDesiredState(speed);
    }

    public void SpinElevator(double speed) {
        m_elevator.SetSpinState(speed);
    }

    public Command ScoreCommand() {
        return runOnce(
          () -> {
            m_elevator.SetDesiredState(0.5);
          }
        );
    }

    public Command MoveElevatorCommand(double speed) {
        return run(
            () -> {
                m_elevator.SetDesiredState(speed);
            }
        );
    }

    public Command SpinElevatorCommand(double speed) {
        return run(
            () -> {
                m_elevator.SetSpinState(speed);
            }
        );
    }

    // private boolean finished;

    // public void ElevatorAuto() {
    //     finished = false;

    //     TimerTask task = new TimerTask() {
    //         public void run() {
    //             finished = true;
    //         }
    //     };

    //     Timer timer = new Timer();

    //     timer.
    // }

    public void ResetElevator() {
        m_elevator.ResetPosition();
    }
}
