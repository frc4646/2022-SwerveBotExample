package frc.team4646;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Updates PIDF of motor controller without needing to program */
public class PIDTuner {
    private static final String DASHBOARD_KEY_P = "Tuner: P";
    private static final String DASHBOARD_KEY_I = "Tuner: I";
    private static final String DASHBOARD_KEY_D = "Tuner: D";
    private static final String DASHBOARD_KEY_F = "Tuner: F";
    private static final String DASHBOARD_KEY_CRACKPOINT = "Tuner: CRACKPOINT";
    private final PID DEFAULT_PID;
    private final double DEFAULT_CRACKPOINT;
    private final int SLOT;
    private final List<TalonFX> motors;
    // private final List<CANSparkMax> motors2;
    private final String name;
    private final String tab;

    private final LinkedHashMap<String, SimpleWidget> widgets;

    /** Updates PIDF of motor controller without needing to program */
    public PIDTuner(String tab, String name, TalonFX... motors) {
        this(tab, name, new PID(), 0.0, 0, motors);
    }

    /** Updates PIDF of motor controller without needing to program */
    public PIDTuner(String tab, String name, PID PID, TalonFX... motors) {
        this(tab, name, PID, 0.0, 0, motors);
    }

    /** Updates PIDF of motor controller without needing to program */
    public PIDTuner(String tab, String name, PID PID, double crackpoint, TalonFX... motors) {
        this(tab, name, PID, crackpoint, 0, motors);
    }

    /** Updates PIDF of motor controller without needing to program */
    public PIDTuner(String tab, String name, PID PID, double crackpoint, int slot, TalonFX... motors) {
        this.motors = List.of(motors);
        // this.motors2 = List.of();
        DEFAULT_PID = PID;
        DEFAULT_CRACKPOINT = crackpoint;
        SLOT = slot;
        this.tab = tab;
        this.name = name;
        
        var valueMap = new LinkedHashMap<String, Double>();
        valueMap.put(DASHBOARD_KEY_P, DEFAULT_PID.P);
        valueMap.put(DASHBOARD_KEY_I, DEFAULT_PID.I);
        valueMap.put(DASHBOARD_KEY_D, DEFAULT_PID.D);
        valueMap.put(DASHBOARD_KEY_F, DEFAULT_PID.F);
        valueMap.put(DASHBOARD_KEY_CRACKPOINT, DEFAULT_CRACKPOINT);
        widgets = ShuffleboardHelpers.Create_TabWithGrids_Editable(tab, name, valueMap);
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
        double P = widgets.get(name + DASHBOARD_KEY_P).getEntry().getDouble(DEFAULT_PID.P);
        double I = widgets.get(name + DASHBOARD_KEY_I).getEntry().getDouble(DEFAULT_PID.I);
        double D = widgets.get(name + DASHBOARD_KEY_D).getEntry().getDouble(DEFAULT_PID.D);
        double F = widgets.get(name + DASHBOARD_KEY_F).getEntry().getDouble(DEFAULT_PID.F);
        
        for (TalonFX motor : motors) {
            TalonUtil.checkError(motor.config_kP(SLOT, P, Constants.CAN_TIMEOUT), "Tuner: could not set P: ");
            TalonUtil.checkError(motor.config_kI(SLOT, I, Constants.CAN_TIMEOUT), "Tuner: could not set I: ");
            TalonUtil.checkError(motor.config_kD(SLOT, D, Constants.CAN_TIMEOUT), "Tuner: could not set D: ");
            TalonUtil.checkError(motor.config_kF(SLOT, F, Constants.CAN_TIMEOUT), "Tuner: could not set F: ");
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
        return SmartDashboard.getNumber(name + DASHBOARD_KEY_CRACKPOINT, 0.0);
    }
}
