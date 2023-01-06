package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {
    
    public final String name;
    public final int driveID;
    public final int turnID;
    public final int cancoderID;
    public Rotation2d offset;

    public SwerveModuleConfig(String name, int driveID, int turnID, int cancoderID, Rotation2d offset) {
        this.name = name;
        this.driveID = driveID;
        this.turnID = turnID;
        this.cancoderID = cancoderID;
        this.offset = offset;
    }
}