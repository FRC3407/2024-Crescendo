package frc.utils;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.Command;

//Intended to be a replacement of the wpilib Trigger class, 
//allowing existing Triggers to be descheduled
//Requires each TriggerRunnable to be polled each tick the condition
//should be checked
public class TriggerRunnable {
  private final BooleanSupplier m_condition;
  private boolean m_pressedLast;
  private final Command m_command;

  // The type of loop to be checked, nearly identical to the wpilib Trigger types
  public static enum LoopType {
    onToggle,
    onTrue,
    onFalse,
    whileTrue,
    whileFalse,
    toggleOnTrue,
    toggleOnFalse,
  }

  // The type of loop checked in poll
  private final LoopType m_loopType;

  /**
   * Creates a new trigger based on the given condition.
   *
   * @param m_loopType  The type of logic to check when polled
   * @param m_condition The BooleanSupplier condition represented by this trigger
   * @param m_command   The command to run when m_condition meets the requirements
   *                    of the loop type
   */
  public TriggerRunnable(LoopType m_loopType, BooleanSupplier m_condition, Command m_command) {
    this.m_command = m_command;
    this.m_loopType = m_loopType;
    this.m_condition = requireNonNullParam(m_condition, "condition", "Trigger");
    m_pressedLast = m_condition.getAsBoolean();
  }

  /**
   * Checks the current supplier value vs its value last tick,
   * and determines what value to return based on the type of loop
   * 
   * @return True if the conditions of the loop type is met
   */
  public boolean poll() {
    if (m_loopType == LoopType.onToggle) {
      return onToggle();
    } else if (m_loopType == LoopType.onTrue) {
      return onTrue();
    } else if (m_loopType == LoopType.onFalse) {
      return onFalse();
    } else if (m_loopType == LoopType.whileTrue) {
      return whileTrue();
    } else if (m_loopType == LoopType.whileFalse) {
      return whileFalse();
    } else if (m_loopType == LoopType.toggleOnFalse) {
      return toggleOnFalse();
    } else if (m_loopType == LoopType.toggleOnTrue) {
      return toggleOnTrue();
    }
    m_pressedLast = m_condition.getAsBoolean();
    return false;
  }

  /**
   * Starts the given command whenever the condition changes
   *
   * @return boolean, true if the command was scheduled
   */
  public boolean onToggle() {
    boolean pressed = m_condition.getAsBoolean();

    if (m_pressedLast != pressed) {
      m_command.schedule();
      return true;
    }
    return false;
  }

  /**
   * Starts the given command whenever the condition changes from `false` to
   * `true`.
   *
   * @return boolean, true if the command was scheduled
   */
  public boolean onTrue() {
    boolean pressed = m_condition.getAsBoolean();

    if (!m_pressedLast && pressed) {
      m_command.schedule();
      return true;
    }
    return false;
  }

  /**
   * Starts the given command whenever the condition changes from `true` to
   * `false`.
   *
   * @return boolean, true if the command was scheduled
   */
  public boolean onFalse() {
    boolean pressed = m_condition.getAsBoolean();

    if (m_pressedLast && !pressed) {
      m_command.schedule();
      return true;
    }
    return false;
  }

  /**
   * Starts the given command whenever the condition changes to `true` and
   * cancels it when it changes to `false`.
   *
   * @return boolean, true if the command was scheduled
   */
  public boolean whileTrue() {
    boolean pressed = m_condition.getAsBoolean();
    if (!m_pressedLast && pressed) {
      m_command.schedule();
      return true;
    } else if (m_pressedLast && !pressed) {
      m_command.cancel();
      return false;
    }
    return false;
  }

  /**
   * Starts the given command whenever the condition changes to `false` and
   * cancels it when it changes to `true`.
   *
   * @return boolean, true if the command was scheduled
   */
  public boolean whileFalse() {
    boolean pressed = m_condition.getAsBoolean();
    if (!m_pressedLast && pressed) {
      m_command.schedule();
      return true;
    } else if (m_pressedLast && !pressed) {
      m_command.cancel();
      return false;
    }
    return false;
  }

  /**
   * Toggles a command when the condition changes from `true` to `false`.
   *
   * @return boolean, true if the command was scheduled
   */
  public boolean toggleOnFalse() {
    boolean pressed = m_condition.getAsBoolean();

    if (m_pressedLast && !pressed) {
      if (m_command.isScheduled()) {
        m_command.cancel();
        return false;
      } else {
        m_command.schedule();
        return true;
      }
    }
    return false;
  }

  /**
   * Toggles a command when the condition changes from `false` to `true`.
   *
   * @return boolean, true if the command was scheduled
   */
  public boolean toggleOnTrue() {
    boolean pressed = m_condition.getAsBoolean();

    if (!m_pressedLast && pressed) {
      if (m_command.isScheduled()) {
        m_command.cancel();
        return false;
      } else {
        m_command.schedule();
        return true;
      }
    }
    return false;
  }
}
