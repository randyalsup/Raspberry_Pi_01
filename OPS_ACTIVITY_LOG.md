# Operations Activity Log

Purpose: Track workspace-level performance optimizations applied for VS Code, enabling easy rollback if needed.

## 2025-12-07

- Added `.vscode/settings.json` to reduce VS Code memory use in this workspace:
  - Disabled telemetry and Git integration in workspace.
  - Disabled minimap and breadcrumbs.
  - Excluded heavy folders from search and file watcher: `node_modules`, `.git`, `.pio`, `.venv`, `build`.
  - Set startup editor to `none` to avoid auto-opening files.

- UI code optimizations in `raspberry_pi/remote/remote_control_ui.py`:
  - Cached small font in `ArrowController` to avoid per-frame font allocations.
  - Switched framebuffer copy path to use NumPy `pygame.surfarray.pixels3d` for zero-copy access, reducing per-frame allocations.

Rollback: Delete `.vscode/settings.json` to restore prior VS Code behavior; revert the above code changes using Git if necessary.