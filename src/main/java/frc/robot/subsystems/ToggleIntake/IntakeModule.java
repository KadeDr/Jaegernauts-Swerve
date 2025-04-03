package frc.robot.subsystems.ToggleIntake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Configs.IntakeConfigs;

public class IntakeModule {
    private final SparkMax m_spark;
    private final SparkClosedLoopController m_CLC;
    private final RelativeEncoder m_encoder;

    private IntakeModuleState m_desiredState = new IntakeModuleState(new Rotation2d());
    
    public IntakeModule(int canId) {
        m_spark = new SparkMax(canId, MotorType.kBrushless);
        m_CLC = m_spark.getClosedLoopController();
        m_encoder = m_spark.getEncoder();
        m_spark.configure(IntakeConfigs.mainConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_desiredState.angle = new Rotation2d(m_encoder.getPosition());
    }

    public Rotation2d getState() {
        return new Rotation2d(m_encoder.getPosition());
    }

    public void SetState(IntakeModuleState desiredState) {
        m_CLC.setReference(desiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }
}
