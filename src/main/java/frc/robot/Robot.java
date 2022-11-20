// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.SwerveDrive.SwerveModule;
import frc.team4646.Util;

public class Robot extends TimedRobot {
   private final Drivetrain m_swerve = new Drivetrain();
 // ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
  //private final SwerveModule m_frontLeft = new SwerveModule(12, 11, 20, shuffleboardTab.getLayout("Front Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0));

  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);
  
  @Override
  public void robotInit() {

  }

  @Override
  public void autonomousPeriodic() {
    // driveWithJoystick(false);
    // m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {

   // m_swerve.drive(m_controller.getLeftX() * Drivetrain.kMaxSpeed, m_controller.getLeftY() * Drivetrain.kMaxSpeed, m_controller.getRightX() * Drivetrain.kMaxAngularSpeed, false);
    driveWithJoystick(false);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
     final var xSpeed =
         -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.07))
             * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = 0;
        //-m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.07))
         //   * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = 0;
       // -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
      //      * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
