// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DealgaefySubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MainCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Vars
  boolean intakeLowered = true;
  boolean usingManual = false;
  boolean usingGyro = true;
  boolean xFalse = true;
  boolean bFalse = true;

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ElevatorSubsytem m_robotElevator = new ElevatorSubsytem();
  private final ClimbSubsystem m_robotClimb = new ClimbSubsystem();
  private final MainCamera m_robotCamera = new MainCamera();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final DealgaefySubsystem m_robotDealgaefy = new DealgaefySubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverCommandController = new CommandXboxController(OIConstants.kDriverControllerPort);
  XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);
  CommandXboxController m_armCommandController = new CommandXboxController(OIConstants.kArmControllerPort);

  // Autonomous Vars
  private final SendableChooser<Command> autoChooser;

  // Score Command for Autonomous
  private final Command getScoreCommand() {
    return Commands.parallel(
        new InstantCommand(() -> m_robotElevator.MoveToLevel4Command()),
        Commands.waitSeconds(0.3)).andThen(
            Commands.parallel(
                new InstantCommand(() -> m_robotElevator.SpinElevator(0.1)),
                Commands.waitSeconds(1)).andThen(
                    new InstantCommand(() -> m_robotElevator.MoveToLevel1Command()),
                    new InstantCommand(() -> m_robotElevator.SpinElevator(0))));
  }

  private final Command getRaiseElevatorCommand() {
    return Commands.parallel(
        new InstantCommand(() -> m_robotElevator.MoveToLevel4Command()));
  }

  private final Command getScoreWithoutRaiseCommand() {
    return Commands.parallel(
        new InstantCommand(() -> m_robotElevator.SpinElevator(0.1)),
        Commands.waitSeconds(1)).andThen(
            new InstantCommand(() -> m_robotElevator.MoveToLevel1Command()),
            new InstantCommand(() -> m_robotElevator.SpinElevator(0)),
            Commands.waitSeconds(0.435));
  }

  private final Command getIntakeCommand() {
    return Commands.parallel(
        new InstantCommand(() -> m_robotElevator.SpinElevator(0.1)),
        Commands.waitSeconds(0.7)).andThen(new InstantCommand(() -> m_robotElevator.SpinElevator(0)));
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("score", getScoreCommand());
    NamedCommands.registerCommand("intake", getIntakeCommand());
    NamedCommands.registerCommand("raise", getRaiseElevatorCommand());
    NamedCommands.registerCommand("score without raise", getScoreWithoutRaiseCommand());

    SmartDashboard.putData(raiseToLevel4Command());

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                usingGyro),
            m_robotDrive));
  }

  public void Arm() {
    boolean l1 = m_armController.getLeftBumperButton();
    boolean r1 = m_armController.getRightBumperButton();
    boolean triangle = m_armController.getYButton();
    boolean cross = m_armController.getAButton();
    double leftTrigger = m_armController.getLeftTriggerAxis();
    double rightTrigger = m_armController.getRightTriggerAxis();
    // int pov = m_armController.getPOV();

    if (leftTrigger >= 0.05) {
      usingManual = true;
      SetElevatorState(-leftTrigger / 4);
    } else if (rightTrigger >= 0.05) {
      usingManual = true;
      SetElevatorState(rightTrigger / 4);
    } else if (rightTrigger <= 0.05 && rightTrigger >= -0.05 && usingManual == true) {
      SetElevatorState(0);
      usingManual = false;
    }

    if (triangle) {
      SetElevatorSpin(0.1);
    } else if (cross) {
      SetElevatorSpin(-0.1);
    } else {
      SetElevatorSpin(0);
    }

    SmartDashboard.putBoolean("Gyro", usingGyro);
  }

  public void SetElevatorState(double speed) {
    m_robotElevator.MoveElevator(speed);
  }

  public void SetElevatorSpin(double speed) {
    m_robotElevator.SpinElevator(speed);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // new Trigger(() -> m_armController.getPOV() == 0)
    // .onTrue(m_robotElevator.MoveToLevel4Command());

    m_armCommandController.povUp()
        .onTrue(new InstantCommand(() -> m_robotElevator.MoveToLevel4Command())
            .andThen(Commands.waitUntil(() -> m_robotElevator.atSetpoint()))
            .andThen(new InstantCommand(() -> System.out.println("I RAISED TO LEVEL 4"))));
    m_armCommandController.povRight().onTrue(new InstantCommand(() -> m_robotElevator.MoveToLevel3Command()));
    m_armCommandController.povDown().onTrue(new InstantCommand(() -> m_robotElevator.MoveToLevel1Command()));
    // m_armCommandController.x().whileTrue(new InstantCommand(() ->
    // m_robotDealgaefy.setDesiredState(0.8)));
    // m_armCommandController.b().whileTrue(new InstantCommand(() ->
    // m_robotDealgaefy.setDesiredState(-0.8)));
    // m_armCommandController.x().onFalse(new InstantCommand(() ->
    // m_robotDealgaefy.setDesiredState(0)));
    // m_armCommandController.b().onFalse(new InstantCommand(() ->
    // m_robotDealgaefy.setDesiredState(0)));
    m_armCommandController.x().onTrue(
        Commands.parallel(m_robotIntake.raiseIntake(), Commands.waitSeconds(2)).andThen(m_robotIntake.lowerIntake()));

    m_driverCommandController.leftBumper().whileTrue(new InstantCommand(() -> m_robotClimb.MoveClimbCommand1(1)));
    m_driverCommandController.leftTrigger().whileTrue(new InstantCommand(() -> m_robotClimb.MoveClimbCommand1(-1)));
    m_driverCommandController.leftBumper().onFalse(new InstantCommand(() -> m_robotClimb.MoveClimbCommand1(0)));
    m_driverCommandController.leftTrigger().onFalse(new InstantCommand(() -> m_robotClimb.MoveClimbCommand1(0)));
    m_driverCommandController.povUp().whileTrue(new InstantCommand(() -> m_robotClimb.MoveClimbCommand2(0.2)));
    m_driverCommandController.povDown().whileTrue(new InstantCommand(() -> m_robotClimb.MoveClimbCommand2(-0.2)));
    m_driverCommandController.povUp().onFalse(new InstantCommand(() -> m_robotClimb.MoveClimbCommand2(0)));
    m_driverCommandController.povDown().onFalse(new InstantCommand(() -> m_robotClimb.MoveClimbCommand2(0)));
    m_driverCommandController.leftBumper().toggleOnTrue(new InstantCommand(() -> usingGyro = !usingGyro));
    m_driverCommandController.povLeft().onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
  }

  public void runCameraCommand() {
    m_robotCamera.CameraTeleopPeriodic(m_driverController.getLeftX());
  }

  public void startCameraCommand() {
    m_robotCamera.CameraRobotPeriodic();
  }

  public void initializeCameraCommand() {
    m_robotCamera.InitializeCamera();
  }

  public SequentialCommandGroup raiseToLevel4Command() {
    return new SequentialCommandGroup(new InstantCommand(() -> m_robotElevator.MoveToLevel4Command()),
        (Commands.waitUntil(() -> m_robotElevator.atSetpoint())),
        (new InstantCommand(() -> System.out.println("I RAISED TO LEVEL 4"))));
  }

  public void initializeField() {
  }

  // public void getAprilTagsCommand(double forward, double strafe, double
  // rotation) {
  // m_robotDrive.drive(forward, strafe, rotation, true);
  // }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // Create config for trajectory
  // TrajectoryConfig config = new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // Trajectory centerRightPath = TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(2.25, 0, new Rotation2d(0)),
  // config);

  // var thetaController = new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand leftSwerveControllerCommand = new
  // SwerveControllerCommand(
  // centerRightPath,
  // m_robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // m_robotDrive::setModuleStates,
  // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(centerRightPath.getInitialPose());

  // m_robotDrive.initialize(centerRightPath);

  // // Run path following command, then stop at the end.
  // return leftSwerveControllerCommand.andThen(
  // () -> m_robotDrive.drive(0, 0, 0, false));
  // // .andThen(
  // // Commands.parallel(
  // // new InstantCommand(() -> m_robotElevator.MoveToLevel4Command()),
  // // Commands.waitSeconds(2)))
  // // .andThen(
  // // Commands.parallel(new InstantCommand(
  // // () -> m_robotElevator.SpinElevator(0.1)),
  // // Commands.waitSeconds(2)))
  // // .andThen(
  // // () -> m_robotElevator.SpinElevator(0))
  // // .andThen(
  // // () -> m_robotElevator.MoveToLevel4AutoCommand());
  // }

  // // public Command testAuto(){
  // // return new SequentialCommandGroup(
  // // // Commands.parallel(new InstantCommand(()->m_robotDrive.drive(.5, 0, 0,
  // // bFalse)),
  // // // new WaitCommand(10)),
  // // new RunCommand(()->m_robotDrive.drive(.5, 0, 0, bFalse)).withTimeout(10),
  // // new InstantCommand(()->m_robotDrive.drive(0, 0, 0, bFalse))

  // // );
  // // }
}
