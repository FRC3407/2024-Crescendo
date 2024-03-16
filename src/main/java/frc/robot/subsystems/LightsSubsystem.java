package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to controll all the lights running on the external lights
 * microcontroller.
 * <p>
 * The whole "lights plan" should be implemented from within this subsystem,
 * controlled mostly from within the {@code periodic()} method.
 * <p>
 * If you need external subsystems and commands to communicate into this class,
 * create "modes" within the class to indicate external states. For instance,
 * if you need a command to indicate when the robot is preparing to shoot, add
 * a {@code setShootingMode(boolean)} to this class. Then, add code to the
 * {@code periodic()} that changes animations based on current modes.
 */
public class LightsSubsystem extends SubsystemBase {

  public static int I2C_ADDRESS = 0x41;
  public static final int MAX_ANIMATIONS = 20;   // Must be 32 or less
  public static final int MAX_STRIPS = 5;        // Must be 8 or less

  private byte[] currentAnimation = new byte[MAX_STRIPS];
  private byte[] nextAnimation = new byte[MAX_STRIPS];
  private byte[] dataOut = new byte[1];
  private Flinger m_flinger;
  private FloorIntake m_intake;

  private I2C i2c = null;

  public LightsSubsystem(Flinger flinger, FloorIntake intake) {
    i2c = new I2C(Port.kOnboard, I2C_ADDRESS);
    m_flinger = flinger;
    m_intake = intake;
    clearAllAnimations();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setAnimation(0, 0);
      setAnimation(1, 0);
      setAnimation(2, 0);
      setAnimation(3, 0);
      setAnimation(4, 0);

    }

    if (DriverStation.isTeleop()) {
      setAnimation(0, 1);
      setAnimation(1, 1);
      setAnimation(2, 1);
      setAnimation(3, 1);
      setAnimation(4, 1);
    }

    if (m_intake.getBotSensor()) {
      setAnimation(0, 2);
      setAnimation(1, 2);
      setAnimation(2, 2);
      setAnimation(3, 2);
      setAnimation(4, 2);
    }

    if (m_flinger.getRPM() > 0) {
      setAnimation(0, 3);
      setAnimation(1, 3);
      setAnimation(2, 3);
      setAnimation(3, 3);
      setAnimation(4, 3);
    }

    // TODO: monitor intrnal robot state and change animations as necessary
    sendAllAnimations();
  }

  /**
   * Clear out all the strips and stop all animation.
   * <br>
   * This should not be called from outside this subsystem.
   */
  protected void clearAllAnimations() {
    for (int s = 0; s < MAX_STRIPS; s++) {
      Integer b = Integer.valueOf(((s << 4) & 0xF0) | (MAX_ANIMATIONS & 0x0F));
      nextAnimation[s] = b.byteValue();
      currentAnimation[s] = (byte) 0;
    }
  }

  /**
   * Set one strip to have the numbered animation.
   * <br>
   * This should not be called from outside this subsystem.
   */
  protected void setAnimation(int stripNumber, int animNumber) {
    Integer b = Integer.valueOf(((stripNumber << 5) & 0xE0) | (animNumber & 0x1F));
    nextAnimation[stripNumber] = b.byteValue();
  }

  /**
   * Push out all animation changes to the Lights Board. <br/>
   * This program takes a <em>lazy</em> approach, in that animation signals are
   * only sent out if they <em>need</em> to change. Signals are only sent if the
   * desired animation is different from the current animation.
   * This prevents redundant, unnecessary changes from dominating the I2C bus.
   */
  private void sendAllAnimations() {
    for (int s = 0; s < MAX_STRIPS; s++) {
      if (nextAnimation[s] != currentAnimation[s]) {
        sendOneAnimation(s);
        currentAnimation[s] = nextAnimation[s];
      }
    }
  }

  private void sendOneAnimation(int stripNumber) {
    dataOut[0] = nextAnimation[stripNumber];
    i2c.writeBulk(dataOut, dataOut.length);
  }
}
