package com.team581.util.state_machines;

import com.team581.util.scheduling.LifecycleSubsystem;
import com.team581.util.scheduling.LifecycleSubsystemManager;
import com.team581.util.scheduling.SubsystemPriorityBase;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;

/** A state machine backed by {@link LifecycleSubsystem}. */
public abstract class StateMachine<S extends Enum<S>> extends LifecycleSubsystem {
  private static final StateMachineInputManager manager = new StateMachineInputManager();

  private S state;
  private boolean isInitialized = false;
  private double lastTransitionTimestamp = Timer.getFPGATimestamp();

  /**
   * Creates a new state machine.
   *
   * @param priority The subsystem priority of this subsystem in {@link LifecycleSubsystemManager}.
   * @param initialState The initial/default state of the state machine.
   */
  protected StateMachine(SubsystemPriorityBase priority, S initialState) {
    super(priority);
    state = initialState;
    manager.register(this);
  }

  /** Processes collecting inputs, state transitions, and state actions. */
  @Override
  public void robotPeriodic() {
    // The first time the robot boots up, we need to set the state from null to the initial state
    // This also gives us an opportunity to run the state actions for the initial state
    // Think of it as transitioning from the robot being off to initialState
    if (!isInitialized) {
      doTransition();
      isInitialized = true;
    }

    setStateFromRequest(getNextState(state));

    whileInState(state);
  }

  /**
   * Gets the current state.
   *
   * @return The current state.
   */
  public S getState() {
    return state;
  }

  /**
   * Creates a command that waits until this state machine is in the given state.
   *
   * @param goalState The state to wait for.
   * @return A command that waits until the state is equal to the goal state.
   */
  public Command waitForState(S goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }

  /**
   * Creates a command that waits until this state machine is in any of the given states.
   *
   * @param goalStates A set of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  public Command waitForStates(Set<S> goalStates) {
    return Commands.waitUntil(() -> goalStates.contains(this.state));
  }

  /**
   * Creates a command that waits until this state machine is in any of the given states.
   *
   * @param goalStates An array of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  @SafeVarargs
  public final Command waitForStates(S... goalStates) {
    return waitForStates(Set.of(goalStates));
  }

  /**
   * {@link StateMachineInputManager} will call this method for each state machine. Used for
   * retrieving sensor values, etc. Inputs are collected in a special phase before any subsystem
   * periodic methods are run.
   *
   * <p>Default behavior is to do nothing.
   */
  protected void collectInputs() {}

  /**
   * Process transitions from one state to another.
   *
   * <p>Default behavior is to stay in the current state indefinitely.
   *
   * @param currentState The current state.
   * @return The new state after processing transitions.
   */
  protected S getNextState(S currentState) {
    return currentState;
  }

  /**
   * Runs once before exiting the current state. This is where you should run state exit actions.
   *
   * <p>Default behavior is to do nothing.
   *
   * @param oldState The state being exited.
   * @param newState The state being entered.
   */
  protected void beforeTransition(S oldState, S newState) {}

  /**
   * Runs once after entering a new state. This is where you should run one-off state actions.
   *
   * @param newState The newly entered state.
   */
  protected void afterTransition(S newState) {}

  /**
   * Called each loop while in the current state. Used for continuous state actions.
   *
   * <p>Default behavior is to do nothing.
   *
   * @param state The current state.
   */
  protected void whileInState(S state) {}

  /**
   * Used to change to a new state when a request is made. Will also trigger all logic that should
   * happen when a state transition occurs.
   *
   * @param requestedState The new state to transition to.
   */
  protected void setStateFromRequest(S requestedState) {
    if (state == requestedState) {
      // No change
      return;
    }

    // Call beforeTransition before changing state
    beforeTransition(state, requestedState);

    state = requestedState;
    doTransition();
  }

  /**
   * Checks if the current state has been in for longer than the given duration. Used for having
   * timeout logic in state transitions.
   *
   * @param duration The timeout duration (in seconds) to use.
   * @return Whether the current state has been active for longer than the given duration.
   */
  protected boolean timeout(double duration) {
    var currentStateDuration = Timer.getFPGATimestamp() - lastTransitionTimestamp;

    return currentStateDuration > duration;
  }

  /** Run side effects that occur when a state transition happens. */
  private void doTransition() {
    DogLog.log(subsystemName + "/State", state);

    lastTransitionTimestamp = Timer.getFPGATimestamp();

    afterTransition(state);
  }
}
