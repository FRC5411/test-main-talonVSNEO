package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private testSubsystem testSub;

  public RobotContainer(boolean isTalon) {
    // Change boolean to test falcon vs NEO
    testSub = new testSubsystem(isTalon);
    configureBindings();
  }

  private void configureBindings() {}

  public void runVoltage(double volts) {
    testSub.setVolts(volts);
  }
  
  public void runVelocityRPM(double vel) {
    testSub.setVelocity(vel);
  }

  public Command getSysIDTests() {
    return testSub.getSysIDTests();
  }
}