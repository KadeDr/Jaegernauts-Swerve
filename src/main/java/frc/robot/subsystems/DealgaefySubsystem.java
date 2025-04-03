package frc.robot.subsystems;

import frc.robot.subsystems.Dealgaefy.DealgaefyModule;

public class DealgaefySubsystem {
    private final DealgaefyModule m_dealgaefy = new DealgaefyModule(14);

    public void setDesiredState(double speed) {
        m_dealgaefy.setDesiredState(speed);
    }
}
