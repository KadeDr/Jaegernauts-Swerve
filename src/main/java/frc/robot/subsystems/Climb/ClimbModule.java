package frc.robot.subsystems.Climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs.ClimbConfig;

public class ClimbModule {
    private final SparkMax spark;
    private final SparkMax spark2;
    
    public ClimbModule(int canID1, int canID2) {
        spark = new SparkMax(canID1, MotorType.kBrushless);
        spark2 = new SparkMax(canID2, MotorType.kBrushless);
        spark.configure(ClimbConfig.mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        spark.configure(ClimbConfig.invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void SetDesiredState1(double speed) {
        spark.set(speed);
    }

    public void SetDesiredState2(double speed) {
        spark2.set(speed);
    }

    public void SetDesiredStateBoth(double speed) {
        spark.set(speed);
        spark2.set(speed);
    }
}
