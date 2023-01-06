package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }
  
    /** This function is called every robot packet, no matter the mode. Use this for items like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating. */
    @Override
    public void robotPeriodic() {
        robotContainer.cacheSensors();
        CommandScheduler.getInstance().run();  // Must be called from robotPeriodic(). Runs these steps: Polls buttons, adds newly-scheduled commands, runs already-scheduled commands, removes finished or interrupted commands, calls subsystem periodic() methods.
        robotContainer.updateHardware();
        robotContainer.updateDashboard();
    }
    
    @Override
    public void disabledInit() {
        robotContainer.onDisable();
        robotContainer.autoModeSelector.reset();
        robotContainer.autoModeSelector.update();
    }
  
    @Override
    public void autonomousInit() {
        robotContainer.onEnable(true);
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }
  
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
           autonomousCommand.cancel();  // If we want auto command to continue until interrupted by another command, comment this line out.
        }
        robotContainer.onEnable(false);
    }
  
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        robotContainer.onEnable(false);
        robotContainer.runTests();
    }
  
    @Override
    public void disabledPeriodic() {
        robotContainer.autoModeSelector.update();
        robotContainer.whileDisabled();
    
        Optional<Command> autoMode = robotContainer.autoModeSelector.getAutoMode();
        if (autoMode.isPresent()) {
            // System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
            autonomousCommand = autoMode.get();
        }
    }
  
    @Override
    public void autonomousPeriodic() {}
  
    @Override
    public void teleopPeriodic() {}
  
    @Override
    public void testPeriodic() {}
}
