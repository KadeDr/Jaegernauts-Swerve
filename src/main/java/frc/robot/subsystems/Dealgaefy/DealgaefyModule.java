package frc.robot.subsystems.Dealgaefy;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs.DealgaefyConfig;

public class DealgaefyModule {
    private final SparkMax spark;
    
    public DealgaefyModule(int canId) {
        spark = new SparkMax(canId, MotorType.kBrushless);

        spark.configure(DealgaefyConfig.mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDesiredState(double speed) {
        spark.set(speed);
    }
}
