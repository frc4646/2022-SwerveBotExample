package frc.team4646;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Updates PIDF of motor controller without needing to program */
public class PIDTuner {
    public static class DashboardConfig {
        public String TabName;
        public String Name;
        public int X;
        public int Y;

        public DashboardConfig(String tabName, String name, int x, int y) {
            TabName = tabName;
            Name = name;
            X = x;
            Y = y;
        }
    }

    private static final String DASHBOARD_KEY_P = "P";
    private static final String DASHBOARD_KEY_I = "I";
    private static final String DASHBOARD_KEY_D = "D";
    private static final String DASHBOARD_KEY_F = "F";
    private static final String DASHBOARD_KEY_CRACKPOINT = "CRACKPOINT";

    private final PID DEFAULT_PID;
    private final double DEFAULT_CRACKPOINT;
    private final int SLOT;

    private final List<TalonFX> motors;
    // private final List<CANSparkMax> motors2;

    private PID currentPID;

    private NetworkTableEntry widgetP;
    private NetworkTableEntry widgetI;
    private NetworkTableEntry widgetD;
    private NetworkTableEntry widgetF;
    private NetworkTableEntry widgetCRACKPOINT;

    /** Updates PIDF of motor controller without needing to program */
    public PIDTuner(DashboardConfig config, TalonFX... motors) {
        this(config, new PID(), 0.0, 0, motors);
    }

    /** Updates PIDF of motor controller without needing to program */
    public PIDTuner(DashboardConfig config, PID PID, TalonFX... motors) {
        this(config, PID, 0.0, 0, motors);
    }

    /** Updates PIDF of motor controller without needing to program */
    public PIDTuner(DashboardConfig config, PID PID, double crackpoint, TalonFX... motors) {
        this(config, PID, crackpoint, 0, motors);
    }

    /** Updates PIDF of motor controller without needing to program */
    public PIDTuner(DashboardConfig config, PID PID, double crackpoint, int slot, TalonFX... motors) {
        this.motors = List.of(motors);
        // this.motors2 = List.of();

        DEFAULT_PID = PID;
        DEFAULT_CRACKPOINT = crackpoint;
        SLOT = slot;

        currentPID = PID;
        
        createDash(config);
    }

    private void createDash(DashboardConfig config) {
        // get the tab and the current count
        var tab = Shuffleboard.getTab(config.TabName);

        // add our grid to it
        var layout = tab.getLayout(config.Name, BuiltInLayouts.kGrid)
            .withSize(1, 3)
            .withPosition(config.X, config.Y)
            .withProperties(Map.of("Number of rows", 5, "Number of columns", 1));

        widgetP = layout.add(DASHBOARD_KEY_P, DEFAULT_PID.P).withPosition(0, 0).getEntry();
        widgetI = layout.add(DASHBOARD_KEY_I, DEFAULT_PID.I).withPosition(0, 1).getEntry();
        widgetD = layout.add(DASHBOARD_KEY_D, DEFAULT_PID.D).withPosition(0, 2).getEntry();
        widgetF = layout.add(DASHBOARD_KEY_F, DEFAULT_PID.F).withPosition(0, 3).getEntry();
        widgetCRACKPOINT = layout.add(DASHBOARD_KEY_CRACKPOINT, DEFAULT_CRACKPOINT).withPosition(0, 4).getEntry();
    }

    /** Updates PIDF of motor controller without needing to program */
    // public PIDTuner(String name, CANSparkMax... motors) {
    //   this(name, new PID(), 0.0, 0, motors);
    // }

    // /** Updates PIDF of motor controller without needing to program */
    // public PIDTuner(String name, PID PID, double crackpoint, CANSparkMax... motors) {
    //   this(name, PID, crackpoint, 0, motors);
    // }

    // /** Updates PIDF of motor controller without needing to program */
    // public PIDTuner(String name, PID PID, double crackpoint, int slot, CANSparkMax... motors) {
    //   this.motors = List.of();
    //   this.motors2 = List.of(motors);
    //   DEFAULT_PID = PID;
    //   DEFAULT_CRACKPOINT = crackpoint;
    //   SLOT = slot;
    //   this.name = name;
    //   initDashboard();
    // }

    /** 
     * Call to refresh motor controller PIDF with values from Smart Dashboard.
     * Suggest from subsystem.OnEnabled() or subsystem.OnDisabled().
     */
    public void updateMotorPIDF() {
        double P = widgetP.getDouble(DEFAULT_PID.P);
        double I = widgetI.getDouble(DEFAULT_PID.I);
        double D = widgetD.getDouble(DEFAULT_PID.D);
        double F = widgetF.getDouble(DEFAULT_PID.F);
        
        // write new PIDF values if changed
        if (!Util.epsilonEquals(currentPID.P, P) || !Util.epsilonEquals(currentPID.I, I) || 
            !Util.epsilonEquals(currentPID.D, D) || !Util.epsilonEquals(currentPID.F, F)) {
            currentPID = new PID(P,I,D,F);
            
            for (TalonFX motor : motors) {
                TalonFXFactory.setPID(motor, currentPID, SLOT);
            }
        }
        // for (CANSparkMax motor : motors2) {
        //   motor.getPIDController().setP(P, SLOT);
        //   motor.getPIDController().setI(I, SLOT);
        //   motor.getPIDController().setD(D, SLOT);
        //   motor.getPIDController().setFF(F, SLOT);
        // }
    }
    
    /** @return crackpoint. Pass into motor controller's set function. */
    public double getCrackpoint() {
        return widgetCRACKPOINT.getDouble(DEFAULT_CRACKPOINT);
    }
}
