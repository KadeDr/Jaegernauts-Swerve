package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
        public static final class ClimbConfig {
                public static final SparkMaxConfig mainConfig = new SparkMaxConfig();
                public static final SparkMaxConfig invertedConfig = new SparkMaxConfig();

                static {
                        mainConfig
                                        .smartCurrentLimit(20)
                                        .inverted(false)
                                        .idleMode(IdleMode.kBrake);

                        invertedConfig
                                        .apply(mainConfig)
                                        .inverted(true);
                }
        }

        public static final class DealgaefyConfig {
                public static final SparkMaxConfig mainConfig = new SparkMaxConfig();

                static {
                        mainConfig
                                        .smartCurrentLimit(20)
                                        .inverted(true)
                                        .idleMode(IdleMode.kBrake);
                }
        }

        public static final class IntakeConfigs {
                public static final SparkMaxConfig mainConfig = new SparkMaxConfig();
                public static final MAXMotionConfig maxConfig = new MAXMotionConfig();

                static {
                        mainConfig
                                        .smartCurrentLimit(20)
                                        .idleMode(IdleMode.kBrake);
                        mainConfig.encoder
                                        .positionConversionFactor(IntakeConstants.kPositionFactor)
                                        .velocityConversionFactor(IntakeConstants.kVelocityFactor);
                        mainConfig.closedLoop
                                        .apply(maxConfig)
                                        .pid(2, 0, 0)
                                        .velocityFF(1 / 473)
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .outputRange(-1, 1);

                        maxConfig
                                        .maxVelocity(IntakeConstants.kVelocityFactor)
                                        .maxAcceleration(IntakeConstants.kVelocityFactor);

                }
        }

        public static final class ElevatorModule {
                public static final SparkMaxConfig mainConfig = new SparkMaxConfig();
                public static final SparkMaxConfig invertedConfig = new SparkMaxConfig();
                public static final SparkMaxConfig spinConfig = new SparkMaxConfig();

                static {
                        @SuppressWarnings("unused")
                        double positionFactor = 1 / ElevatorConstants.kPositionFactor;
                        @SuppressWarnings("unused")
                        double velocityFactor = 3;

                        mainConfig
                                        .smartCurrentLimit(80)
                                        .idleMode(IdleMode.kBrake);
                        mainConfig.absoluteEncoder
                                        .positionConversionFactor(ElevatorConstants.kEncoderRotationsPerInch)
                                        .velocityConversionFactor(ElevatorConstants.kMaxSpeedInchesPerSecond);
                        mainConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(0.15, 0, 0) // P = .15 for not MAXMotion
                                        .velocityFF(1 / 473)
                                        .outputRange(-1, 1);
                        mainConfig.closedLoop.maxMotion
                                        .maxAcceleration(7500)
                                        .maxVelocity(2000000)
                                        .allowedClosedLoopError(0.5)
                                        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

                        invertedConfig
                                        .apply(mainConfig)
                                        .inverted(true);

                        spinConfig
                                        .inverted(true);
                }
        }

        public static final class MAXSwerveModule {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward gain.
                        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                                        / ModuleConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;
                        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor) // meters
                                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(0.04, 0, 0)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(-1, 1);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20);
                        turningConfig.absoluteEncoder
                                        // Invert the turning encoder, since the output shaft rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .inverted(true)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(1, 0, 0)
                                        .outputRange(-1, 1)
                                        // Enable PID wrap around for the turning motor. This will allow the PID
                                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                                        // to 10 degrees will go through 0 rather than the other direction which is a
                                        // longer route.
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
