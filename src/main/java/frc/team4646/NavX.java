package frc.team4646;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;

public class NavX extends AHRS{
    
    public NavX() {
        super(SPI.Port.kMXP);
    }

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Gyro");
		builder.addDoubleProperty("Value", super::getYaw, null);
    }

    @Override
    public Rotation2d getRotation2d() {
        return super.getRotation2d().unaryMinus();
    }
}
