# AutoMan_Project

This workspace now simulates the case-study policy of keeping automation focused
on repetitive, simple dismantling tasks while routing complex work to human
specialists.

Current simulation behavior:

- `perception/vision_pipeline.py` dispatches only simple repetitive dismantling
  steps to the robot cell.
- Complex or non-repetitive tasks such as battery extraction and PCB triage are
  explicitly routed to humans.
- `dual_ur5/simple_task_executor.py` simulates dual-arm execution and reports
  task completion back to the perception node.
- The simulation summary logs the estimated productivity gain from this
  selective automation strategy.
