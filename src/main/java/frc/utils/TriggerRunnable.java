package frc.utils;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.Command;

//Intended to be a replacement of the wpilib Trigger class, 
//allowing existing Triggers to be essentially deschedule
//Requires each TriggerRunnable to be polled each tick the condition
//should be checked
public class TriggerRunnable {
  private final BooleanSupplier m_condition;
  private boolean m_pressedLast;
  private final Command command;

  //The type of loop to be checked, nearly identical to the wpilib Trigger types
  public static enum LoopType {
    onTrue,
    onFalse,
    whileTrue,
    whileFalse,
    toggleOnTrue,
    toggleOnFalse
  }

  //The type of loop checked in poll
  private final LoopType loopType;

  /**
   * Creates a new trigger based on the given condition.
   *
   * @param loop      The loop instance that polls this trigger.
   * @param condition The condition represented by this trigger
   */
  public TriggerRunnable(LoopType loopType, BooleanSupplier condition, Command command) {
    this.command = command;
    this.loopType = loopType;
    m_condition = requireNonNullParam(condition, "condition", "Trigger");
    m_pressedLast = m_condition.getAsBoolean();
  }

  /**
   * Checks the current supplier value vs its value last tick, 
   * and determines what value to return based on the type of loop
   * @return True if the conditions of the loop type is met
   */
  public boolean poll() {
    if (loopType == LoopType.onTrue) {
      return onTrue();
    } else if (loopType == LoopType.onFalse) {
      return onFalse();
    } else if (loopType == LoopType.whileTrue) {
      return whileTrue();
    } else if (loopType == LoopType.whileFalse) {
      return whileFalse();
    } else if (loopType == LoopType.toggleOnFalse) {
      return toggleOnFalse();
    } else if (loopType == LoopType.toggleOnTrue) {
      return toggleOnTrue();
    }
    m_pressedLast = m_condition.getAsBoolean();
    return false;
  }

  /**
   * Starts the given command whenever the condition changes from `false` to
   * `true`.
   *
   * @param command the command to start
   * @return boolean, true if the command was scheduled
   */
  public boolean onTrue() {
    boolean pressed = m_condition.getAsBoolean();

    if (!m_pressedLast && pressed) {
      command.schedule();
      return true;
    }
    return false;
  }

  /**
   * Starts the given command whenever the condition changes from `true` to
   * `false`.
   *
   * @param command the command to start
   * @return boolean, true if the command was scheduled
   */
  public boolean onFalse() {
    boolean pressed = m_condition.getAsBoolean();

    if (m_pressedLast && !pressed) {
      command.schedule();
      return true;
    }
    return false;
  }

  /**
   * Starts the given command whenever the condition changes to `true` and
   * cancels it when it changes to `false`.
   *
   * @param command the command to start
   * @return boolean, true if the command was scheduled
   */
  public boolean whileTrue() {
    boolean pressed = m_condition.getAsBoolean();
    if (!m_pressedLast && pressed) {
      command.schedule();
      return true;
    } else if (m_pressedLast && !pressed) {
      command.cancel();
      return false;
    }
    return false;
  }

  /**
   * Starts the given command whenever the condition changes to `false` and
   * cancels it when it changes to `true`.
   *
   * @param command the command to start
   * @return boolean, true if the command was scheduled
   */
  public boolean whileFalse() {
    boolean pressed = m_condition.getAsBoolean();
    if (!m_pressedLast && pressed) {
      command.schedule();
      return true;
    } else if (m_pressedLast && !pressed) {
      command.cancel();
      return false;
    }
    return false;
  }

  /**
   * Toggles a command when the condition changes from `true` to `false`.
   *
   * @param command the command to toggle
   * @return boolean, true if the command was scheduled
   */
  public boolean toggleOnFalse() {
    boolean pressed = m_condition.getAsBoolean();

    if (m_pressedLast && !pressed) {
      if (command.isScheduled()) {
        command.cancel();
        return false;
      } else {
        command.schedule();
        return true;
      }
    }
    return false;
  }

  /**
   * Toggles a command when the condition changes from `false` to `true`.
   *
   * @param command the command to toggle
   * @return boolean, true if the command was scheduled
   */
  public boolean toggleOnTrue() {
    boolean pressed = m_condition.getAsBoolean();

    if (!m_pressedLast && pressed) {
      if (command.isScheduled()) {
        command.cancel();
        return false;
      } else {
        command.schedule();
        return true;
      }
    }
    return false;
  }
}
