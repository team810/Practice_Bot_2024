package frc.lib;

import edu.wpi.first.wpilibj.XboxController;

public class StadiaController extends XboxController {
	public StadiaController(int port) {
		super(port);
	}

	@Override
	public double getLeftX() {
		return super.getRawAxis(0);
	}

	@Override
	public double getRightX() {
		return super.getRawAxis(3);
	}

	@Override
	public double getLeftY() {
		return super.getRawAxis(1);
	}

	@Override
	public double getRightY() {

		return super.getRawAxis(4);
	}

	@Override
	public boolean getLeftBumper() {return getRawButton(5);}

	@Override
	public boolean getRightBumper() {return getRawButton(6);}

	@Override
	public boolean getAButton() {return super.getAButton();}

	@Override
	public boolean getBButton() {return super.getBButton();}

	@Override
	public boolean getXButton() {return super.getXButton();}

	@Override
	public boolean getYButton() {return super.getYButton();}

}
