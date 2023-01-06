// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.team4646.Util;

public class teleopDrive extends CommandBase {
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);

    private boolean fieldRelative = false;
    /** Creates a new teleopDrive. */
    SwerveDriveSubsystem theDrivetrain;
    XboxController theXbox;

    public teleopDrive(SwerveDriveSubsystem drivetrain, XboxController Xbox) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        theDrivetrain = drivetrain;
        theXbox = Xbox;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(Util.handleDeadband(theXbox.getLeftY(), 0.2)) * Constants.SWERVE_DRIVE.MAX_SPEED;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(Util.handleDeadband(theXbox.getLeftX(), 0.2)) * Constants.SWERVE_DRIVE.MAX_SPEED;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(Util.handleDeadband(theXbox.getRightX(), 0.2)) * Constants.SWERVE_DRIVE.MAX_ANGULAR_SPEED;

        if (theXbox.getAButtonPressed()) {
            fieldRelative = !fieldRelative;
        }

        theDrivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);  
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
