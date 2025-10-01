// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  Field2d field;

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The robot's Gyro
  private final JaegernautsNavXGyro m_gyro = JaegernautsNavXGyro.getInstance();

  private final Field2d m_field = new Field2d();

  // public void initialize(Trajectory trajectory) {
  // SmartDashboard.putData("Field", m_field);
  // m_field.getObject("traj").setTrajectory(trajectory);
  // setOdymetry(9.51);
  // }

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveWithFeedforwards,
        // ChassisSpeeds. Also optionally outputs individual module
        // feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(8.3, 0.0, 0.125), // Translation PID constants
            new PIDConstants(8.3, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
    // // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Odyometry Angle", m_odometry.getPoseMeters().getRotation().getDegrees());
    field.setRobotPose(getPose());
    m_field.setRobotPose(getPose());
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  public void setOdymetry(double pos) {
    m_frontLeft.setPosition(pos);
    m_frontRight.setPosition(pos);
    m_rearLeft.setPosition(pos);
    m_rearRight.setPosition(pos);
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };

    return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered, ySpeedDelivered, rotDelivered,
            Rotation2d.fromDegrees(-m_gyro.getAngle()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Calculate and apply feedforward per module
    m_frontLeft.setDesiredState(
        swerveModuleStates[0],
        DriveConstants.kDriveFeedforward.calculate(swerveModuleStates[0].speedMetersPerSecond));

    m_frontRight.setDesiredState(
        swerveModuleStates[1],
        DriveConstants.kDriveFeedforward.calculate(swerveModuleStates[1].speedMetersPerSecond));

    m_rearLeft.setDesiredState(
        swerveModuleStates[2],
        DriveConstants.kDriveFeedforward.calculate(swerveModuleStates[2].speedMetersPerSecond));

    m_rearRight.setDesiredState(
        swerveModuleStates[3],
        DriveConstants.kDriveFeedforward.calculate(swerveModuleStates[3].speedMetersPerSecond));
  }

  public void driveWithFeedforwards(ChassisSpeeds speeds, DriveFeedforwards ff) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    double[] feedforwards = ff.linearForcesNewtons();

    // Now you can see each module’s raw force
    SmartDashboard.putNumber("FF Front Left (N)", feedforwards[0]);
    SmartDashboard.putNumber("FF Front Right (N)", feedforwards[1]);
    SmartDashboard.putNumber("FF Rear Left (N)", feedforwards[2]);
    SmartDashboard.putNumber("FF Rear Right (N)", feedforwards[3]);

    // --- APPLY YOUR OWN FF (Option A: keep WPILib SimpleMotorFeedforward) ---
    m_frontLeft.setDesiredState(
        swerveModuleStates[0],
        DriveConstants.kDriveFeedforward.calculate(swerveModuleStates[0].speedMetersPerSecond));

    m_frontRight.setDesiredState(
        swerveModuleStates[1],
        DriveConstants.kDriveFeedforward.calculate(swerveModuleStates[1].speedMetersPerSecond));

    m_rearLeft.setDesiredState(
        swerveModuleStates[2],
        DriveConstants.kDriveFeedforward.calculate(swerveModuleStates[2].speedMetersPerSecond));

    m_rearRight.setDesiredState(
        swerveModuleStates[3],
        DriveConstants.kDriveFeedforward.calculate(swerveModuleStates[3].speedMetersPerSecond));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        DriveConstants.kDriveFeedforward.calculate(0));
    m_frontRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        DriveConstants.kDriveFeedforward.calculate(0));
    m_rearLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        DriveConstants.kDriveFeedforward.calculate(0));
    m_rearRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        DriveConstants.kDriveFeedforward.calculate(0));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Desaturate wheel speeds so no module exceeds max velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Apply feedforward per module just like in drive()
    m_frontLeft.setDesiredState(
        desiredStates[0],
        DriveConstants.kDriveFeedforward.calculate(desiredStates[0].speedMetersPerSecond));

    m_frontRight.setDesiredState(
        desiredStates[1],
        DriveConstants.kDriveFeedforward.calculate(desiredStates[1].speedMetersPerSecond));

    m_rearLeft.setDesiredState(
        desiredStates[2],
        DriveConstants.kDriveFeedforward.calculate(desiredStates[2].speedMetersPerSecond));

    m_rearRight.setDesiredState(
        desiredStates[3],
        DriveConstants.kDriveFeedforward.calculate(desiredStates[3].speedMetersPerSecond));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
