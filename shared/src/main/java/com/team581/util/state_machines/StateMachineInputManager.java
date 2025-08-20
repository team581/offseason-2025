package com.team581.util.state_machines;

import com.team581.util.scheduling.LifecycleSubsystem;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;

/** Helps ensure that state machines can collect inputs before executing state actions. */
public class StateMachineInputManager extends LifecycleSubsystem {
  // Sort by lowest priority first
  private final Queue<StateMachine<?>> stateMachines =
      new PriorityQueue<>(
          Comparator.comparingInt(stateMachine -> stateMachine.priority.getValue()));

  public StateMachineInputManager() {
    // Ensures that state machine inputs are gathered at the right time
    // Subsystem inputs are collected in reverse order of priority (so lowest priority first)
    super(() -> 999);
  }

  public void register(StateMachine<?> stateMachine) {
    stateMachines.add(stateMachine);
  }

  @Override
  public void robotPeriodic() {
    for (var stateMachine : stateMachines) {
      stateMachine.collectInputs();
    }
  }
}
