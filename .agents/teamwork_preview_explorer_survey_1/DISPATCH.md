## 2026-09-15T09:04:36Z
You are teamwork_preview_explorer_survey_1.
Your working directory is: d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_1
Your dispatch file is: d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_1\DISPATCH.md
MANDATORY: Read the original user request at: d:\Newfolder\1234\.agents\ORIGINAL_REQUEST.md before starting work.

Your task is to investigate the codebase for Requirements R1, R2, R3, and R5:
1. R1: Analyze `SortingController.execute_sort()` and `_move_and_wait()` in `src/control/sorting_controller.py`. How START_AUTO is triggered, how joint targets are sent, how motion completion is polled, and how to restructure to PC Master model (`send_joint_targets` -> `send_command(cfg.plc.commands.move)` -> poll `motion_done` from `read_status()` with timeout -> `write_bit(*ADDR.GRIP)`). Detail how to isolate PLC autonomous sequence into `execute_sort_plc_sequence()`.
2. R2: Inspect `PLCController.send_joint_targets()` in `src/plc/plc_controller.py`. Detail byte offsets, DB mapping, how `j4_target == -1` causes buffer write over bytes 54-57 (`theta_j2_ht`), and how to dynamically assemble buffer (12 bytes vs 16 bytes).
3. R3: Centralized joint limits (`config.yaml`, `src/kinematics/`). How to centralize `JOINT_LIMITS` into `src/kinematics/joint_limits.py`, raise `WorkspaceError` when limits are exceeded in `inverse_kinematics()`, and handle it in `SortingController._ik()`.
4. R5: Thread churn in `PLCController.send_pulse()`. Detail `threading.Timer` usage and design for dedicated worker queue / UI-safe timer dispatch with clean cancellation.

Write your findings and recommendations in:
`d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_1\handoff.md`
and notify me via send_message when done.
