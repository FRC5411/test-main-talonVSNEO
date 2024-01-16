package frc.robot;

public class RobotContainer {
  testSubsystem testSub;

  public RobotContainer(boolean isTalon) {
    testSub = new testSubsystem(false);
    configureBindings();
  }

  private void configureBindings() {}

  public void runVoltage(double volts) {
    testSub.setVolts(volts);
  }
  
  public void runVelocityRPM(double vel) {
    testSub.setVelocity(vel);
  }

  public void runSysID() {
    testSub.runSysIDTest();
  }
}
