package frc.utils;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.utils.ClearableEventLoop;

public class TriggerRunnable {
  private final BooleanSupplier m_condition;
  private boolean m_pressedLast = false;
  private final Command command;

  public static enum LoopType {
    onTrue,
    onFalse,
  }

  private final LoopType loopType;

  /**
   * Creates a new trigger based on the given condition.
   *
   * @param loop      The loop instance that polls this trigger.
   * @param condition the condition represented by this trigger
   */
  public TriggerRunnable(LoopType loopType, BooleanSupplier condition, Command command) {
    this.command = command;
    this.loopType = loopType;
    m_condition = requireNonNullParam(condition, "condition", "Trigger");
  }

  public boolean poll() {
    if (loopType == LoopType.onTrue) {
      return onTrue();
    } else if (loopType == LoopType.onFalse) {
      return onFalse();
    }
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
      m_pressedLast = pressed;
      command.schedule();
      return true;
    }

    m_pressedLast = pressed;
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
      m_pressedLast = pressed;
      command.schedule();
      return true;
    }

    m_pressedLast = pressed;
    return false;
  }

  // /**
  // * Starts the given command whenever the condition changes from `true` to
  // * `false`.
  // *
  // * @param command the command to start
  // * @return this trigger, so calls can be chained
  // */
  // public Trigger onFalse(Command command) {
  // requireNonNullParam(command, "command", "onFalse");
  // m_loop.bind(
  // new Runnable() {
  // private boolean m_pressedLast = m_condition.getAsBoolean();

  // @Override
  // public void run() {
  // boolean pressed = m_condition.getAsBoolean();

  // if (m_pressedLast && !pressed) {
  // command.schedule();
  // }

  // m_pressedLast = pressed;
  // }
  // });
  // return getAsTrigger();
  // }

  // /**
  // * Starts the given command when the condition changes to `true` and cancels
  // it
  // * when the condition
  // * changes to `false`.
  // *
  // * <p>
  // * Doesn't re-start the command if it ends while the condition is still
  // `true`.
  // * If the command
  // * should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
  // *
  // * @param command the command to start
  // * @return this trigger, so calls can be chained
  // */
  // public Trigger whileTrue(Command command) {
  // requireNonNullParam(command, "command", "whileTrue");
  // m_loop.bind(
  // new Runnable() {
  // private boolean m_pressedLast = m_condition.getAsBoolean();

  // @Override
  // public void run() {
  // boolean pressed = m_condition.getAsBoolean();

  // if (!m_pressedLast && pressed) {
  // command.schedule();
  // } else if (m_pressedLast && !pressed) {
  // command.cancel();
  // }

  // m_pressedLast = pressed;
  // }
  // });
  // return getAsTrigger();
  // }

  // /**
  // * Starts the given command when the condition changes to `false` and cancels
  // it
  // * when the
  // * condition changes to `true`.
  // *
  // * <p>
  // * Doesn't re-start the command if it ends while the condition is still
  // `false`.
  // * If the command
  // * should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
  // *
  // * @param command the command to start
  // * @return this trigger, so calls can be chained
  // */
  // public Trigger whileFalse(Command command) {
  // requireNonNullParam(command, "command", "whileFalse");
  // m_loop.bind(
  // new Runnable() {
  // private boolean m_pressedLast = m_condition.getAsBoolean();

  // @Override
  // public void run() {
  // boolean pressed = m_condition.getAsBoolean();

  // if (m_pressedLast && !pressed) {
  // command.schedule();
  // } else if (!m_pressedLast && pressed) {
  // command.cancel();
  // }

  // m_pressedLast = pressed;
  // }
  // });
  // return getAsTrigger();
  // }

  // /**
  // * Toggles a command when the condition changes from `false` to `true`.
  // *
  // * @param command the command to toggle
  // * @return this trigger, so calls can be chained
  // */
  // public Trigger toggleOnTrue(Command command) {
  // requireNonNullParam(command, "command", "toggleOnTrue");
  // m_loop.bind(
  // new Runnable() {
  // private boolean m_pressedLast = m_condition.getAsBoolean();

  // @Override
  // public void run() {
  // boolean pressed = m_condition.getAsBoolean();

  // if (!m_pressedLast && pressed) {
  // if (command.isScheduled()) {
  // command.cancel();
  // } else {
  // command.schedule();
  // }
  // }

  // m_pressedLast = pressed;
  // }
  // });
  // return getAsTrigger();
  // }

  // /**
  // * Toggles a command when the condition changes from `true` to `false`.
  // *
  // * @param command the command to toggle
  // * @return this trigger, so calls can be chained
  // */
  // public Trigger toggleOnFalse(Command command) {
  // requireNonNullParam(command, "command", "toggleOnFalse");
  // m_loop.bind(
  // new Runnable() {
  // private boolean m_pressedLast = m_condition.getAsBoolean();

  // @Override
  // public void run() {
  // boolean pressed = m_condition.getAsBoolean();

  // if (m_pressedLast && !pressed) {
  // if (command.isScheduled()) {
  // command.cancel();
  // } else {
  // command.schedule();
  // }
  // }

  // m_pressedLast = pressed;
  // }
  // });
  // return getAsTrigger();
  // }

  // public Trigger getAsTrigger() {
  // return new Trigger(m_loop, m_condition);
  // }

  // @Override
  // public boolean getAsBoolean() {
  // return m_condition.getAsBoolean();
  // }

  // /**
  // * Composes two triggers with logical AND.
  // *
  // * @param trigger the condition to compose with
  // * @return A trigger which is active when both component triggers are active.
  // */
  // public Trigger and(BooleanSupplier trigger) {
  // return new Trigger(() -> m_condition.getAsBoolean() &&
  // trigger.getAsBoolean());
  // }

  // /**
  // * Composes two triggers with logical OR.
  // *
  // * @param trigger the condition to compose with
  // * @return A trigger which is active when either component trigger is active.
  // */
  // public Trigger or(BooleanSupplier trigger) {
  // return new Trigger(() -> m_condition.getAsBoolean() ||
  // trigger.getAsBoolean());
  // }

  // /**
  // * Creates a new trigger that is active when this trigger is inactive, i.e.
  // that
  // * acts as the
  // * negation of this trigger.
  // *
  // * @return the negated trigger
  // */
  // public Trigger negate() {
  // return new Trigger(() -> !m_condition.getAsBoolean());
  // }

  // /**
  // * Creates a new debounced trigger from this trigger - it will become active
  // * when this trigger has
  // * been active for longer than the specified period.
  // *
  // * @param seconds The debounce period.
  // * @return The debounced trigger (rising edges debounced only)
  // */
  // public Trigger debounce(double seconds) {
  // return debounce(seconds, Debouncer.DebounceType.kRising);
  // }

  // /**
  // * Creates a new debounced trigger from this trigger - it will become active
  // * when this trigger has
  // * been active for longer than the specified period.
  // *
  // * @param seconds The debounce period.
  // * @param type The debounce type.
  // * @return The debounced trigger.
  // */
  // public Trigger debounce(double seconds, Debouncer.DebounceType type) {
  // return new Trigger(
  // new BooleanSupplier() {
  // final Debouncer m_debouncer = new Debouncer(seconds, type);

  // @Override
  // public boolean getAsBoolean() {
  // return m_debouncer.calculate(m_condition.getAsBoolean());
  // }
  // });
  // }

  // // /**
  // // * Debinds all runnables from m_loop
  // // */
  // // public void debind()
  // // {
  // // m_loop.clear();
  // // }
}
