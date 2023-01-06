package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SDSModuleConfig {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    public final boolean canCoderInvert;

    public SDSModuleConfig(double wheelDiameter, double angleGearRatio, double driveGearRatio, boolean driveMotorInvert, boolean angleMotorInvert, boolean canCoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.canCoderInvert = canCoderInvert;
    }

    public static SDSModuleConfig SDSMK4i(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        boolean driveMotorInvert = true;
        boolean angleMotorInvert = true;
        boolean canCoderInvert = false;
        return new SDSModuleConfig(wheelDiameter, angleGearRatio, driveGearRatio, driveMotorInvert, angleMotorInvert, canCoderInvert);
    }

    /* Drive Gear Ratios for Mk4i */
    public class Mk4i{
        /** SDS MK4i - 8.14 : 1 */
        public static final double L1 = (8.14 / 1.0);
        /** SDS MK4i - 6.75 : 1 */
        public static final double L2 = (6.75 / 1.0);
        /** SDS MK4i - 6.12 : 1 */
        public static final double L3 = (6.12 / 1.0);
    }
}
