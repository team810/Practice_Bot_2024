package frc.robot.util;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Pneumatics extends SubsystemBase {
    private final static Pneumatics INSTANCE = new Pneumatics();
    public static Pneumatics getInstance() {
        return INSTANCE;
    }

    private final PneumaticsModuleType pneumaticsModuleType = PneumaticsModuleType.REVPH;

    private final PneumaticHub revPneumaticsHub;
    private final REVPHSim revPneumaticsHubSim;

    private final PneumaticsControlModule ctrePneumaticsControlModule;
    private final CTREPCMSim ctrePnumaticsControlModuleSim;

    private final Compressor compressor;

    private Pneumatics() {
        switch (pneumaticsModuleType)
        {
            case CTREPCM:
                ctrePneumaticsControlModule = new PneumaticsControlModule();
                if (Robot.isSimulation()){ctrePnumaticsControlModuleSim = new CTREPCMSim(ctrePneumaticsControlModule);} else {ctrePnumaticsControlModuleSim = null;}

                revPneumaticsHub = null;
                revPneumaticsHubSim = null;

                compressor = ctrePneumaticsControlModule.makeCompressor();
                break;
            case REVPH:
                ctrePneumaticsControlModule = null;
                ctrePnumaticsControlModuleSim = null;

                revPneumaticsHub = new PneumaticHub();
                if (Robot.isSimulation()) {revPneumaticsHubSim = new REVPHSim(revPneumaticsHub);}else{revPneumaticsHubSim = null;}

                compressor = revPneumaticsHub.makeCompressor();
                break;
            default:
                ctrePneumaticsControlModule = null;
                ctrePnumaticsControlModuleSim = null;

                revPneumaticsHub = null;
                revPneumaticsHubSim = null;
                compressor = null;
                throw new RuntimeException("Pneumatics Hub type not defined properly, it needs to be ether a ctre or rev Pneumatics module");
        }
    }

    public Solenoid createSolenoid(int channel)
    {
        switch (pneumaticsModuleType)
        {
            case CTREPCM:
                return ctrePneumaticsControlModule.makeSolenoid(channel);
            case REVPH:
                return revPneumaticsHub.makeSolenoid(channel);
            default:
                throw new RuntimeException("Pneumatics Hub type not defined properly, it needs to be ether a ctre or rev Pneumatics module");
        }
    }

    public DoubleSolenoid createSolenoid(int fwd, int back)
    {
        switch (pneumaticsModuleType)
        {
            case CTREPCM:
                return ctrePneumaticsControlModule.makeDoubleSolenoid(fwd,back);
            case REVPH:
                return revPneumaticsHub.makeDoubleSolenoid(fwd, back);
            default:
                throw new RuntimeException("Pneumatics Hub type not defined properly, it needs to be ether a ctre or rev Pneumatics module");
        }
    }

    public Compressor getCompressor()
    {
        return compressor;
    }

    @Override
    public void periodic() {
        switch (pneumaticsModuleType)
        {
            case CTREPCM:
                break;
            case REVPH:
                break;
            default:
                throw new RuntimeException("Pneumatics Hub type not defined properly, it needs to be ether a ctre or rev Pneumatics module");
        }
    }
}

