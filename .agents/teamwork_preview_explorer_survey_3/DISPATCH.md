## 2026-09-15T09:04:36Z
You are teamwork_preview_explorer_survey_3.
Your working directory is: d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_3
Your dispatch file is: d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_3\DISPATCH.md
MANDATORY: Read the original user request at: d:\Newfolder\1234\.agents\ORIGINAL_REQUEST.md before starting work.

Your task is to investigate the codebase for Requirement R6 and the test suite:
1. R6 UI/UX:
   - Interactive ROI Selection over camera feed / sliders, saving to `config.yaml`.
   - 2D Digital Twin Visualizer on Tkinter Canvas: forward kinematics link/joint poses, Elbow-Up/Down state for Manual and Auto modes.
   - AI Preprocessing Config: `camera.enable_unsharp_mask` boolean flag in `config.yaml` (default true) + bypass logic; ONNX/TensorRT export documentation.
   - i18n Localization: `src/ui/i18n.py` dictionary system for `vi` and `en`, consistent terminology across pages.
2. Test Suite & Infrastructure:
   - Inventory existing tests in `tests/` and test helpers/fixtures.
   - Check `test_gui_no_plc.py` and mock PLC behaviors.
   - Plan new unit tests and E2E verification requirements.

Write your findings and recommendations in:
`d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_3\handoff.md`
and notify me via send_message when done.
