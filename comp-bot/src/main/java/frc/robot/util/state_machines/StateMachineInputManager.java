package frc.robot.util.state_machines;

import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;

/** Helps ensure that state machines can collect inputs before executing state actions. */
public class StateMachineInputManager extends LifecycleSubsystem {
  // Sort by lowest priority first
  private final Queue<StateMachine<?>> stateMachines =
      new PriorityQueue<>(Comparator.comparingInt(stateMachine -> stateMachine.priority.value));

  public StateMachineInputManager() {
    super(SubsystemPriority.STATE_MACHINE_INPUT_MANAGER);
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
