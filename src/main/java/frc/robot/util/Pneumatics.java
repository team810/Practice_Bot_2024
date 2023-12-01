package frc.robot.util;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

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

    private final ArrayList<Integer> activeChannels = new ArrayList<>();

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
        activeChannels.add(channel);

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
        activeChannels.add(fwd);
        activeChannels.add(back);

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
                if (Robot.isReal())
                {
                    Logger.getInstance().recordOutput("Pneumatics/Pressure", compressor.getCurrent());
                    Logger.getInstance().recordOutput("Pneumatics/CompressorOn", ctrePneumaticsControlModule.getCompressor());
                    Logger.getInstance().recordOutput("Pneumatics/CompressorCurrent", compressor.getCurrent());

                    for (int i = 0; i < activeChannels.size(); i++) {
                        Logger.getInstance().recordOutput("Pneumatics/Channel_" + activeChannels.get(i), ctrePneumaticsControlModule.checkSolenoidChannel(activeChannels.get(i)));
                    }
                } else if (Robot.isSimulation()) {
                    Logger.getInstance().recordOutput("Pneumatics/CompressorOn", ctrePnumaticsControlModuleSim.getCompressorOn());
                    Logger.getInstance().recordOutput("Pneumatics/CompressorCurrent", ctrePnumaticsControlModuleSim.getCompressorCurrent());

                    for (int i = 0; i < activeChannels.size(); i++) {
                        Logger.getInstance().recordOutput("Pneumatics/Channel_" + activeChannels.get(i), revPneumaticsHub.checkSolenoidChannel(activeChannels.get(i)));
                    }
                }
                break;
            case REVPH:
                if (Robot.isReal())
                {
                    Logger.getInstance().recordOutput("Pneumatics/Pressure", compressor.getPressure());
                    Logger.getInstance().recordOutput("Pneumatics/CompressorOn", revPneumaticsHub.getCompressor());
                    Logger.getInstance().recordOutput("Pneumatics/CompressorCurrent", compressor.getCurrent());

                    for (int i = 0; i < activeChannels.size(); i++) {
                        Logger.getInstance().recordOutput("Pneumatics/Channel_" + activeChannels.get(i), revPneumaticsHub.checkSolenoidChannel(activeChannels.get(i)));
                    }
                } else if (Robot.isSimulation()) {
                    Logger.getInstance().recordOutput("Pneumatics/CompressorOn", revPneumaticsHubSim.getCompressorOn());
                    Logger.getInstance().recordOutput("Pneumatics/CompressorCurrent", revPneumaticsHubSim.getCompressorCurrent());

                    for (int i = 0; i < activeChannels.size(); i++) {
                        Logger.getInstance().recordOutput("Pneumatics/Channel_" + activeChannels.get(i), revPneumaticsHubSim.getSolenoidOutput(activeChannels.get(i)));
                    }
                }
                break;
            default:
                throw new RuntimeException("Pneumatics Hub type not defined properly, it needs to be ether a ctre or rev Pneumatics module");
        }
    }
}

