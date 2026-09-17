# Dispatch to Explorer 2: Vision Pipeline, StabilityTracker & Auto-Pick Integration

## Working Directory
d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_2

## Context & Inputs
- User Request: d:\Newfolder\1234\.agents\ORIGINAL_REQUEST.md
- Relevant Source Files:
  - `src/ai/stability_tracker.py`
  - `src/ai/detector.py`
  - `src/camera/`
  - `src/ui/main_window.py`
  - `main.py`
  - `src/control/sorting_controller.py`

## Mission & Objectives
Investigate the codebase for Requirement R4:
1. Examine `src/ai/stability_tracker.py`: methods, state transitions (`LOCKED`, `SEARCHING`, etc.), threshold parameters, smoothing algorithms, and `draw_lock_overlay()` capabilities.
2. Examine the camera feed and vision thread architecture (in `src/camera/`, `main.py`, or `src/ui/`): how frames are captured, processed by YOLO detector, and passed to UI.
3. Determine how to cleanly integrate `StabilityTracker` into the frame-processing loop: feeding detection coordinates per frame, updating tracker state, rendering lock overlays.
4. Investigate the Continuous Auto-Pick workflow:
   - Where to place the toggle in GUI (default OFF).
   - How triggering `execute_sort()` upon reaching `LOCKED` state should be coordinated to prevent duplicate triggering or racing.
   - How manual button "CHỤP & PHÂN LOẠI" remains functional and safe.

## Output
Produce a detailed handoff report in `d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_2\handoff.md` with:
- Concrete line numbers and code references.
- Exact flow of coordinates from YOLO -> StabilityTracker -> UI overlay -> Auto-Pick trigger.
- Race condition analysis and debouncing strategy.
