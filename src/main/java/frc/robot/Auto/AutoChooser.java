package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Configs;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;

public class AutoChooser {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public AutoChooser(DriveSubsystem drive, ElevatorSubsytem lift) {
        // score, raise, score without raise, intake
        NamedCommands.registerCommand("score",
                Commands.sequence(new InstantCommand(() -> lift.MoveToLevel4Command()),
                        lift.SpinElevatorCommand(0.1).withTimeout(2), lift.SpinElevatorCommand(0),
                        new InstantCommand(() -> lift.MoveToLevel1Command())));

        NamedCommands.registerCommand("raise", new InstantCommand(() -> lift.MoveToLevel4Command()));

        NamedCommands.registerCommand("score without raise",
                Commands.sequence(lift.SpinElevatorCommand(0.1).withTimeout(2), lift.SpinElevatorCommand(0),
                        new InstantCommand(() -> lift.MoveToLevel1Command())));

        NamedCommands.registerCommand("intake",
                Commands.sequence(lift.SpinElevatorCommand(0.1).withTimeout(0.5), lift.SpinElevatorCommand(0)));

        createAutoBuilder(drive);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static void createAutoBuilder(DriveSubsystem drive) {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                drive::getPose,
                drive::resetOdometry,
                drive::getChassisSpeeds,
                (speeds, feedforwards) -> drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false),
                new PPHolonomicDriveController(
                    new PIDConstants(0.04, 0, 0),
                    new PIDConstants(1, 0, 0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drive);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }
}
