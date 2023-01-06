package frc.robot.controls;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoModeSelector {

  enum DesiredMode {
    DO_NOTHING,
  }

  private SendableChooser<DesiredMode> modeSelector;
  private DesiredMode modeDesiredCached = DesiredMode.DO_NOTHING;
  private Optional<Command> mode = Optional.empty();

  public AutoModeSelector() {
    modeSelector = new SendableChooser<>();
    modeSelector.addOption("Do Nothing", DesiredMode.DO_NOTHING);
    SmartDashboard.putData("Auto: Mode", modeSelector);
  }

  private Optional<Command> getAutoModeForParams(DesiredMode mode) {
    switch (mode) {
      case DO_NOTHING:
        return Optional.of(new WaitCommand(15.0));
      default:
        System.err.println("No valid auto mode found for  " + mode);
        // return Optional.of(new ModeFallback());
        return Optional.of(new WaitCommand(15.0));
    }
  }  

  public void update() {
    DesiredMode desiredMode = modeSelector.getSelected();

    if (desiredMode == null) {
      desiredMode = DesiredMode.DO_NOTHING;
    }

    if (modeDesiredCached != desiredMode) {
      System.out.println("Auto: " + desiredMode.name());
      mode = getAutoModeForParams(desiredMode);
    }
    modeDesiredCached = desiredMode;
  }

  public void reset() {
    mode = Optional.empty();
    modeDesiredCached = null;
  }

  public Optional<Command> getAutoMode() {
    return mode;
  }
}
