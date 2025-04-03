package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorModule;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsytem extends SubsystemBase {
    private final ElevatorModule m_elevator = new ElevatorModule(ElevatorConstants.kLeftCANId, ElevatorConstants.kRightCANId, ElevatorConstants.kIntakeCANId);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", m_elevator.getPosition());
    }

    public void MoveElevator(double position) {
        m_elevator.SetDesiredState(position);
    }

    public void MoveElevatorToPosition(double position) {
        m_elevator.SetDesiredPosition(position);
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

    public Command MoveElevatorCommand(double position) {
        return run(
            () -> {
                m_elevator.SetDesiredState(position);
            }
        );
    }

    public void MoveToLevel1Command() {
        m_elevator.MoveToLevel1();
    }

    public void MoveToLevel4Command() {
        m_elevator.MoveToLevel4();
    }

    public void MoveToLevel3Command() {
        m_elevator.MoveToLevel3();
    }

    public void MoveToLevel4AutoCommand() {
        m_elevator.MoveToLevel4Auto();
    }

    public void MoveToLevel4Auto2Command() {
        m_elevator.MoveToLevel4Auto2();
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
        m_elevator.ResetEncoders();
    }
}
