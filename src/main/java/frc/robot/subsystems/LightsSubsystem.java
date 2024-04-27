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

  public static int CIRCLE1 = 0;
  public static int CIRCLE2 = 1;
  public static int CIRCLE3 = 2;

  public static int LINMATRIX = 3;
  public static int LINMATRIX1 = 4;
  public static int LINMATRIX2 = 5;

  public static int MATRIX = 6;

  public static int CONWAYS = 7;

  public static int ORANGE_REVERSE_MATRIX = 8;
  public static int ORANGE_REVERSE_MATRIX1 = 9;
  public static int ORANGE_REVERSE_MATRIX2 = 10;
  public static int ORANGE_REVERSE_MATRIX3 = 11;

  public static int FLASH = 12;

  public static int FILLRED = 13;
  public static int FILLGREEN = 14;
  public static int FILLWHITE = 15;

  public static int BITMAP = 16;

  public static int POINTER = 17;

  public static int TIME_WARNING = 18;

  public static int I2C_ADDRESS = 0x41;

  public static int PERIMETERID = 0;
  public static int BIGPID = 1;
  public static int SIDEID = 2;
  public static int HEADID = 3;
  public static int BACKID = 4;

  public static final int MAX_ANIMATIONS = 31; // Must be 31 or less
  public static final int MAX_STRIPS = 5; // Must be 8 or less

  public static int TIME_WARNING_SECONDS = 20; // Start the time warning 20 seconds before the end of the match

  private byte[] currentAnimation = new byte[MAX_STRIPS];
  private byte[] nextAnimation = new byte[MAX_STRIPS];
  private byte[] dataOut = new byte[1];
  private Flinger m_flinger;
  private FloorIntake m_intake;

  private I2C i2c = null;

  public boolean hasDoneTimeWarning = false;

  public VisionSubsystem m_VisionSubsystem;

  public LightsSubsystem(Flinger flinger, FloorIntake intake, VisionSubsystem vision) {
    i2c = new I2C(Port.kOnboard, I2C_ADDRESS);
    m_flinger = flinger;
    m_intake = intake;
    m_VisionSubsystem = vision;
    clearAllAnimations();
  }

  private boolean isNoteLoaded() {
    return m_intake.getBotSensor();
  }

  private boolean isFlingerRunning() {
    return Math.abs(m_flinger.getRPM_1()) > 0;
  }

  private boolean isIntakeRunning() {
    return Math.abs(m_intake.getMotorSpeed()) > 0;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setAnimation(PERIMETERID, LINMATRIX); // linear_matrix.py
      setAnimation(BIGPID, CONWAYS); // conways_game_of_life.py
      setAnimation(HEADID, LINMATRIX1); // linear_matrix.py
      setAnimation(BACKID, LINMATRIX2); // linear_matrix.py
      setAnimation(SIDEID, MATRIX); // matrix.py

    } else {

      // if (DriverStation.getMatchTime() < 30) {
      //   setAnimation(PERIMETERID, FILLRED); // Fill.py
      //   setAnimation(BIGPID, FILLRED); // Fill.py
      //   setAnimation(HEADID, FILLRED); // Fill.py
      //   setAnimation(BACKID, FILLRED); // Fill.py
      //   setAnimation(SIDEID, FILLRED); // Fill.py
      // }
      if (isNoteLoaded()) {
        setAnimation(PERIMETERID, FLASH); // Flash.py
        setAnimation(BIGPID, FLASH); // Flash.py
        setAnimation(HEADID, FLASH); // Flash.py
        setAnimation(BACKID, FLASH); // Flash.py
        setAnimation(SIDEID, FLASH); // Flash.py
      }
      else if (isIntakeRunning()) {
        setAnimation(PERIMETERID, CIRCLE1); // circle_spinner.py
        setAnimation(BIGPID, ORANGE_REVERSE_MATRIX); // orange_reverse_matrix.py
        setAnimation(HEADID, CIRCLE2); // circle_spinner.py
        setAnimation(BACKID, CIRCLE3); // circle_spinner.py
        setAnimation(SIDEID, ORANGE_REVERSE_MATRIX1); // orange_reverse_matrix.py
      } else if (isFlingerRunning()) {
        setAnimation(BIGPID, ORANGE_REVERSE_MATRIX2); // orange_reverse_matrix.py
        setAnimation(SIDEID, POINTER); // pointer.py
      } else {
        setAnimation(PERIMETERID, FILLGREEN); // Fill.py
        setAnimation(BIGPID, BITMAP); // animation_bitmap.py
        setAnimation(HEADID, FILLWHITE); // Fill.py
        setAnimation(BACKID, FILLRED); // Fill.py
        setAnimation(SIDEID, FILLGREEN); // Fill.py
      }
      
    }

    if (m_VisionSubsystem.isTagVisible(3)) {
      setAnimation(PERIMETERID, FILLGREEN); // FILLGREEN.py
      setAnimation(BIGPID,      FILLGREEN); // FILLGREEN.py
      setAnimation(HEADID,      FILLGREEN); // FILLGREEN.py
      setAnimation(BACKID,      FILLGREEN); // FILLGREEN.py
      setAnimation(SIDEID,      FILLGREEN); // FILLGREEN.py
    }if (m_VisionSubsystem.isTagVisible(15)) {
      setAnimation(PERIMETERID, FILLWHITE); // FILLGREEN.py
      setAnimation(BIGPID,      FILLWHITE); // FILLGREEN.py
      setAnimation(HEADID,      FILLWHITE); // FILLGREEN.py
      setAnimation(BACKID,      FILLWHITE); // FILLGREEN.py
      setAnimation(SIDEID,      FILLWHITE); // FILLGREEN.py
    }

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
      currentAnimation[s] = (byte) MAX_ANIMATIONS;
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

    int stripNumbr = (nextAnimation[stripNumber] & 0xE0) >> 5;
    int animNumber = nextAnimation[stripNumber] & 0x1F;
    System.out.println("I2C: SEND(" + stripNumbr + "," + animNumber + ")");
  }
}
