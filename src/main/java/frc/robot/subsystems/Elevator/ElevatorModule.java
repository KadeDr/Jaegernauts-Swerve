package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

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
        return new ElevatorModuleState(m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    }

    public double getPosition() {
        return m_leftEncoder.getPosition();
    }

    public void SetDesiredPosition(double position) {
        m_leftCLC.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        m_rightCLC.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
   }

    public void SetDesiredState(double speed) {
         m_leftSpark.set(speed);
         m_rightSpark.set(speed);
    }

    public void SetSpinState(double speed) {
        m_spinSpark.set(speed);
    }

    public void MoveToLevel1() {
        m_leftCLC.setReference(ElevatorConstants.Level1, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        m_rightCLC.setReference(ElevatorConstants.Level1, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    public void MoveToLevel4() {
        m_leftCLC.setReference(ElevatorConstants.Level4, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        m_rightCLC.setReference(ElevatorConstants.Level4, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    public void MoveToLevel3() {
        m_leftCLC.setReference(ElevatorConstants.Level3, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        m_rightCLC.setReference(ElevatorConstants.Level3, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    public void MoveToLevel4Auto() {
        m_leftCLC.setReference(ElevatorConstants.Level4Auto, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        m_rightCLC.setReference(ElevatorConstants.Level4Auto, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    public void MoveToLevel4Auto2() {
        m_leftCLC.setReference(ElevatorConstants.Level4Auto2, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        m_rightCLC.setReference(ElevatorConstants.Level4Auto2, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    public void ResetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }
}
