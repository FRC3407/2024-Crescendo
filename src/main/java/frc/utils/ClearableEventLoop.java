package frc.utils;

import java.util.Collection;
import java.util.ConcurrentModificationException;
import java.util.LinkedHashSet;

/**
 * A declarative way to bind a set of actions to a loop and execute them when the loop is polled.
 */
public class ClearableEventLoop {
  private final Collection<Runnable> m_bindings = new LinkedHashSet<>();
  private boolean m_running;

  /** Default constructor. */
  public ClearableEventLoop() {}

  /**
   * Bind a new action to run when the loop is polled.
   *
   * @param action the action to run.
   */
  public void bind(Runnable action) {
    if (m_running) {
      throw new ConcurrentModificationException("Cannot bind EventLoop while it is running");
    }
    m_bindings.add(action);
  }

  /** Poll all bindings. */
  public void poll() {
    try {
      m_running = true;
      m_bindings.forEach(Runnable::run);
    } finally {
      m_running = false;
    }
  }

  /** Clear all bindings. */
  public void clear() {
    if (m_running) {
      throw new ConcurrentModificationException("Cannot clear EventLoop while it is running");
    }
    m_bindings.clear();
  }
}
