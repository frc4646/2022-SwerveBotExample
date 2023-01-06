// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team4646.NavX;
import frc.team4646.PIDTuner;
import frc.team4646.SmartSubsystem;
import frc.team4646.Util;
import frc.team4646.PIDTuner.DashboardConfig;

/** Represents a swerve drive style drivetrain. */
public class SwerveDriveSubsystem extends SmartSubsystem {

    private static final String DASH_NAME = "Swerve";
    private static final String MODULES_DASH_NAME = "Swerve Modules";
    private final Field2d field = new Field2d();

    private final Translation2d[] locations = new Translation2d[] {
        new Translation2d( Constants.SWERVE_DRIVE.WHEEL_TRACK_WIDTH_X_METERS/2.0,  Constants.SWERVE_DRIVE.WHEEL_TRACK_WIDTH_Y_METERS/2.0), // frontLeft
        new Translation2d( Constants.SWERVE_DRIVE.WHEEL_TRACK_WIDTH_X_METERS/2.0, -Constants.SWERVE_DRIVE.WHEEL_TRACK_WIDTH_Y_METERS/2.0), // frontRight
        new Translation2d(-Constants.SWERVE_DRIVE.WHEEL_TRACK_WIDTH_X_METERS/2.0,  Constants.SWERVE_DRIVE.WHEEL_TRACK_WIDTH_Y_METERS/2.0), // backLeft
        new Translation2d(-Constants.SWERVE_DRIVE.WHEEL_TRACK_WIDTH_X_METERS/2.0, -Constants.SWERVE_DRIVE.WHEEL_TRACK_WIDTH_Y_METERS/2.0)  // backRight
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations[0], locations[1], locations[2], locations[3]);

    private final SwerveModule[] modules = new SwerveModule[] {
        new SwerveModule(Constants.SWERVE_DRIVE.CONFIG_FRONT_LEFT),
        new SwerveModule(Constants.SWERVE_DRIVE.CONFIG_FRONT_RIGHT),
        new SwerveModule(Constants.SWERVE_DRIVE.CONFIG_BACK_LEFT),
        new SwerveModule(Constants.SWERVE_DRIVE.CONFIG_BACK_RIGHT) 
    };

    private final NavX gyro = new NavX();
    private boolean gyroInitialized = false;

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
    
    private boolean fieldRelative;
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private boolean drivetrainDashEnabled = false;
    private NetworkTableEntry moduleDashEnWidget;
    private boolean moduleDashEnabled = false;
    private PIDTuner drivePIDTuner;
    private PIDTuner steerPIDTuner;

    public SwerveDriveSubsystem() {
        SmartDashboard.putData("Field", field);
    }

    /**
     * Handles updating changing steering angle offsets, PID tuning, gyro initialization, and odometry
     */
    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.periodic();
        }
        
        // wait for the gyro to be detected before initializing once
        if (!gyroInitialized && gyro.isConnected()) {
            resetGyro();
            gyroInitialized = true;
        }

        updateOdometry();
        field.setRobotPose(odometry.getPoseMeters());

        
        if (!drivetrainDashEnabled) {
            if(Constants.SWERVE_DRIVE.DASHBOARD_ENABLED) {
                createDrivetrainDashboard();
            }   
        }
        // add the PID tuning dashboard if desired
        if (!moduleDashEnabled) {
            if (moduleDashEnWidget != null && moduleDashEnWidget.getBoolean(false)) {
                createModuleDash();
            }
        } else {
            // ensure these exist first
            if(drivePIDTuner != null) drivePIDTuner.updateMotorPIDF();
            if(steerPIDTuner != null) steerPIDTuner.updateMotorPIDF();
        }
    }


    /**
     * Create a dashboard for the drivetrain basics
     */
    private void createDrivetrainDashboard() {
        drivetrainDashEnabled = true;
        var tab = Shuffleboard.getTab(DASH_NAME);
        tab.getComponents().clear();

        tab.addBoolean("Field Relative", () -> fieldRelative).withPosition(0, 0).withSize(2,2);
        tab.addNumber("Cmd: Rot", () -> Util.round(chassisSpeeds.omegaRadiansPerSecond, 2)).withPosition(0, 2).withSize(2,1);
        tab.addNumber("Cmd: X Speed", () -> Util.round(chassisSpeeds.vxMetersPerSecond, 2)).withPosition(0, 3).withSize(1,1);
        tab.addNumber("Cmd: Y Speed", () -> Util.round(chassisSpeeds.vyMetersPerSecond, 2)).withPosition(1, 3).withSize(1,1);

        tab.add("Gyro", gyro).withProperties(Map.of("Counter Clockwise", true)).withPosition(2, 0).withSize(3,3); // set it be CCW+
        tab.addBoolean("Gyro Initialized", () -> gyroInitialized).withPosition(2, 3).withSize(1,1);
        Pose2d pose = odometry.getPoseMeters();
        tab.addString("Pose", () -> String.format("(%.2f, %.2f), %.2f deg", pose.getX(), pose.getY(), pose.getRotation().getDegrees())).withPosition(3, 3).withSize(2,1);

        tab.add("Field", field).withPosition(4, 1).withSize(4, 3);
        
        moduleDashEnWidget = tab.add("Enable Modules Tab", moduleDashEnabled).withPosition(5, 0).withSize(2,1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }

    private void createModuleDash() {
        moduleDashEnabled = true;
        var modulesTab = Shuffleboard.getTab(MODULES_DASH_NAME);
        modulesTab.getComponents().clear();

        // add the 4 modules
        int x = 0;
        for (SwerveModule module : modules) {
            module.createDashboardGrid(modulesTab, x);
            x += 2;
        }

        // add the 2 tuners
        drivePIDTuner = new PIDTuner(new DashboardConfig(MODULES_DASH_NAME, "Drive", x, 0), Constants.SWERVE_DRIVE.DRIVE_PID,
                modules[0].getDriveMotor(),
                modules[1].getDriveMotor(),
                modules[2].getDriveMotor(),
                modules[3].getDriveMotor());
        steerPIDTuner = new PIDTuner(new DashboardConfig(MODULES_DASH_NAME, "Steer", x+1, 0), Constants.SWERVE_DRIVE.STEER_PID,
                modules[0].getSteerMotor(),
                modules[1].getSteerMotor(),
                modules[2].getSteerMotor(),
                modules[3].getSteerMotor());
    }

    /**
     * Drive the robot using joystick info.
     * @param xSpeed   Speed of the robot in the x direction (forward).
     * @param ySpeed   Speed of the robot in the y direction (sideways).
     * @param rot      Angular rate of the robot.
     * @param fieldRel Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRel) {
        // convert the x/y/rot inputs to actual chassis commands depending on field relativity
        fieldRelative = fieldRel;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroAngle());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // use trig to calculate each module's desired states
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        // scale back the speeds to our max achievable
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SWERVE_DRIVE.MAX_SPEED);

        // set each module's desired state
        for (int i = 0; i < desiredStates.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    /**
     * Reset the gyro angle to our current position
     */
    public void resetGyro() {
        gyro.reset();
        odometry.resetPosition(new Pose2d(), getGyroAngle());
    }

    /**
     * Get the gyro angle in the correct orientation to the robot
     * @return Gyro angle, CCW+
     */
    private Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        odometry.update(
                getGyroAngle(), // this may need to be inverted
                modules[0].getState(),
                modules[1].getState(),
                modules[2].getState(),
                modules[3].getState());
    }
}
