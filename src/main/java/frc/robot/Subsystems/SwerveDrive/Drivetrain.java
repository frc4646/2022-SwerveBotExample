// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Commands.teleopDrive;
import frc.team4646.SmartSubsystem;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SmartSubsystem {
    public static final double kMaxSpeed = 2; // 4.5 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final Translation2d m_frontLeftLoc = new Translation2d(0.325, 0.295);
    private final Translation2d m_frontRightLoc = new Translation2d(0.325, -0.295);
    private final Translation2d m_backLeftLoc = new Translation2d(-0.325, 0.295);
    private final Translation2d m_backRightLoc = new Translation2d(-0.325, -0.295);

    private final SwerveModule m_frontLeft = new SwerveModule("Front Left", 11, 12, 13, 0);
    private final SwerveModule m_frontRight = new SwerveModule("Front Right", 14, 15, 16, 232);
    private final SwerveModule m_backLeft = new SwerveModule("Back Left", 20, 21, 22, 0);
    private final SwerveModule m_backRight = new SwerveModule("Back Right", 17, 18, 19, 153);

    private final AHRS m_gyro = new AHRS();

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    public Drivetrain() {
        m_gyro.reset();
    }

    public void resetGyro() {
        m_gyro.reset();
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

        ChassisSpeeds chassisSpeeds;

        SmartDashboard.putBoolean("fieldRelative", fieldRelative);
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroAngle());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Get the gyro angle in the correct orientation to the robot
     */
    private Rotation2d getGyroAngle() {
        return m_gyro.getRotation2d().unaryMinus();
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

    @Override
    public void whileDisabled() {
        m_frontLeft.whileDisabled();
    }
}
