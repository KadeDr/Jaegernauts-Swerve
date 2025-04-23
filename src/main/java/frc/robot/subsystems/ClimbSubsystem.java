package frc.robot.subsystems;

import frc.robot.subsystems.Climb.ClimbModule;

public class ClimbSubsystem {
    private final ClimbModule m_climb = new ClimbModule(13, 15);

    public void MoveClimbCommand1(double speed) {
        m_climb.SetDesiredState1(speed);
    }

    public void MoveClimbCommand2(double speed) {
        m_climb.SetDesiredState2(speed);
    }

    public void MoveClimbCommandBoth(double speed) {
        m_climb.SetDesiredStateBoth(speed);
    }
}
