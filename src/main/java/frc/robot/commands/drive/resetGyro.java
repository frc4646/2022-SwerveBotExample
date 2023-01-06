package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class resetGyro extends InstantCommand {
    private SwerveDriveSubsystem theDrivetrain;

    public resetGyro(SwerveDriveSubsystem drivetrain) {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
      theDrivetrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        theDrivetrain.resetGyro();
    }
}
