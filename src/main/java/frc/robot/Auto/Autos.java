package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Autos {
    public static Command OneCoral(DriveSubsystem drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "1-Coral")
                .setMaxVelocity(4.460)
                .setAcceleration(3.5)
                .build()
        );
    }

    public static Command CenterLeftCoral(DriveSubsystem drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "Center-Left 3-Coral")
                .setMaxVelocity(4.460)
                .setAcceleration(3.5)
                .build()
        );
    }

    public static Command CenterRightCoral(DriveSubsystem drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "Center-Right 3-Coral")
                .setMaxVelocity(4.460)
                .setAcceleration(3.5)
                .build()
        );
    }

    public static Command LeftCoral(DriveSubsystem drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "Left 3-Coral")
                .setMaxVelocity(4.460)
                .setAcceleration(3.5)
                .build()
        );
    }

    public static Command RightCoral(DriveSubsystem drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "Right 3-Coral")
                .setMaxVelocity(4.460)
                .setAcceleration(3.5)
                .build()
        );
    }

    private Autos() {}
}
