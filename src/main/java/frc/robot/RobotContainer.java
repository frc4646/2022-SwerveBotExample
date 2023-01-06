package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.resetGyro;
import frc.robot.commands.drive.teleopDrive;
import frc.robot.controls.AutoModeSelector;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.team4646.SmartSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final SwerveDriveSubsystem SWERVE;
    
    private final List<SmartSubsystem> allSubsystems = new ArrayList<SmartSubsystem>();

    // Controllers and Operator Interface
    private final XboxController driverController = new XboxController(0);

    public AutoModeSelector autoModeSelector;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // create each subsystems
        SWERVE = new SwerveDriveSubsystem();

        // add all subsystems to our list
        allSubsystems.add(SWERVE);

        // setup default commands for each subsystem
        SWERVE.setDefaultCommand(new teleopDrive(SWERVE, driverController));

        // Finally, bind buttons to commands
        configureButtonBindings();
        
        autoModeSelector = new AutoModeSelector();
    }

    /** connect controller buttons to commands */
    private void configureButtonBindings() {
        // Reset Gyro
        new JoystickButton(driverController, XboxController.Button.kY.value).whenPressed(new resetGyro(SWERVE));
        // TODO or all in one line?
        // new JoystickButton(driverController, XboxController.Button.kY.value).whenPressed(new InstantCommand(swerveSubsystem::resetGyro, swerveSubsystem));
    }

    
    public void cacheSensors() {
        allSubsystems.forEach(SmartSubsystem::cacheSensors);
    }

    public void updateHardware() {
        allSubsystems.forEach(SmartSubsystem::updateHardware);
    }

    public void updateDashboard() {
        // boolean isCompetition =  DriverStation.isFMSAttached();
        // boolean isForceButtonPressed = SmartDashboard.getBoolean(Constants.SHOW_DETAILS, false);
        // boolean showDetails = !isCompetition || isForceButtonPressed;
        // allSubsystems.forEach(s -> s.updateDashboard(showDetails));
        // allSubsystems.forEach(s -> SmartDashboard.putData("Subsystem" + s.getName(), s));
    }

    public void onEnable(boolean isAutonomous) {
        allSubsystems.forEach(s -> s.onEnable(isAutonomous));
    }

    public void onDisable() {
        allSubsystems.forEach(SmartSubsystem::onDisable);
        // new DriveDisabled().schedule();
    }
    
    public void whileDisabled() {
        allSubsystems.forEach(SmartSubsystem::whileDisabled);
        // new DriveDisabled().schedule();
    }

    public void runTests() {
        // if (VISION != null) {
        //     VISION.setLED(LEDMode.BLINK);
        // }
        // Test.reset();
        // Timer.delay(3.0);
        // if (VISION != null) {
        //     VISION.setLED(LEDMode.OFF);
        // }
        // allSubsystems.forEach(SmartSubsystem::runTests);
        // Test.results();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
