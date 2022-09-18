// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4646.TalonFXFactory;

public class Robot extends TimedRobot {


  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
 
  private final TalonFX motor11 = new TalonFX(11);

  private final XboxController m_controller = new XboxController(0);
  // private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    // driveWithJoystick(false);
    // m_swerve.updateOdometry();


  }

  @Override
  public void teleopPeriodic() {
    // driveWithJoystick(true);

    double speed = m_controller.getLeftY();
    Rotation2d angle = Rotation2d.fromDegrees( m_controller.getLeftX() );

    SwerveModuleState desiredState = new SwerveModuleState(speed, angle);

    m_frontLeft.setDesiredState(desiredState);

    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("angle", angle.getDegrees());


    motor11.set(ControlMode.Velocity, speed*1000);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    // final var xSpeed =
    //     -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
    //         * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    // final var ySpeed =
    //     -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
    //         * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // final var rot =
    //     -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
    //         * Drivetrain.kMaxAngularSpeed;

    // m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
