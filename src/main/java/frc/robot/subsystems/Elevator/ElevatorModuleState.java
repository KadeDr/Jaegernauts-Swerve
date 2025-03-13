package frc.robot.subsystems.Elevator;

public class ElevatorModuleState {
    public double leftPosition;
    public double rightPosition;

    public ElevatorModuleState() {}
    public ElevatorModuleState(double leftPosition, double rightPosition) {
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
    }
}
