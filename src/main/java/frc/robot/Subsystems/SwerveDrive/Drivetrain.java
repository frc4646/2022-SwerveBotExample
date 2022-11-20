// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 4.5; // 4.5 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.325, 0.295);
  private final Translation2d m_frontRightLocation = new Translation2d(0.325, -0.295);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.325, 0.295);
  private final Translation2d m_backRightLocation = new Translation2d(-0.325, -0.295);

  ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

  private final SwerveModule m_frontLeft = new SwerveModule(11, 12, 13, -10.7, shuffleboardTab.getLayout("Front Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0));
  private final SwerveModule m_frontRight = new SwerveModule(14, 15, 16, 189.5, shuffleboardTab.getLayout("Front Right", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0));
  private final SwerveModule m_backLeft = new SwerveModule(20, 21, 22, 335.9, shuffleboardTab.getLayout("Back Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0));
  private final SwerveModule m_backRight = new SwerveModule(17, 18, 19, 180.5, shuffleboardTab.getLayout("Back Right", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0));

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }
}
