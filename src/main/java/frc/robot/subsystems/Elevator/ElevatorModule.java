package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;

public class ElevatorModule {
    private final SparkMax m_leftSpark, m_rightSpark, m_spinSpark;
    private final RelativeEncoder m_leftEncoder, m_rightEncoder;
    private final SparkClosedLoopController m_leftCLC, m_rightCLC;

    private ElevatorModuleState m_desiredState = new ElevatorModuleState(0, 0);

    public ElevatorModule(int leftCANId, int rightCANId, int spinCANId) {
        m_leftSpark = new SparkMax(leftCANId, MotorType.kBrushless);
        m_rightSpark = new SparkMax(rightCANId, MotorType.kBrushless);
        m_spinSpark = new SparkMax(spinCANId, MotorType.kBrushless);

        m_leftEncoder = m_leftSpark.getEncoder();
        m_rightEncoder = m_rightSpark.getEncoder();

        m_leftCLC = m_leftSpark.getClosedLoopController();
        m_rightCLC = m_rightSpark.getClosedLoopController();

        m_leftSpark.configure(Configs.ElevatorModule.mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightSpark.configure(Configs.ElevatorModule.invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_spinSpark.configure(Configs.ElevatorModule.spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_desiredState.leftPosition = m_leftEncoder.getPosition();
        m_desiredState.rightPosition = m_rightEncoder.getPosition();
    }

    public ElevatorModuleState GetPosition() {
        return m_desiredState;
    }

    public void SetDesiredState(double speed) {
        System.out.println("Moving");
        System.out.println(speed);
       // m_leftCLC.setReference(speed, ControlType.kPosition);
       // m_rightCLC.setReference(speed, ControlType.kPosition);
         m_leftSpark.set(speed);
         m_rightSpark.set(speed);
        System.out.println(m_leftSpark.get());
    }

    public void SetSpinState(double speed) {
        m_spinSpark.set(speed);
    }

    public void ResetPosition() {
        m_leftCLC.setReference(0, ControlType.kPosition);
        m_rightCLC.setReference(0, ControlType.kPosition);
    }
}
