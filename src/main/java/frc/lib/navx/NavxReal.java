package frc.lib.navx;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;

public class NavxReal implements Navx{
	private final AHRS navx = new AHRS(I2C.Port.kMXP);
	public NavxReal()
	{
		navx.reset();
		navx.zeroYaw();
	}
	@Override
	public Rotation2d getRotation2d() {
		Rotation2d rot = new Rotation2d();
		return navx.getRotation2d();
	}

	@Override
	public void zeroYaw() {
		navx.zeroYaw();
	}
	@Override
	public void setOffset(double offset) {
		navx.setAngleAdjustment(offset);
	}
}
