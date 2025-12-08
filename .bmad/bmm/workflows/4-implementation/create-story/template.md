# Story {{epic_num}}.{{story_num}}: {{story_title}}

Status: drafted

## Story

As a {{role}},
I want {{action}},
so that {{benefit}}.

## Acceptance Criteria

1. [Add acceptance criteria from epics/PRD]

## Tasks / Subtasks

- [ ] Task 1 (AC: #)
  - [ ] Subtask 1.1
- [ ] Task 2 (AC: #)
  - [ ] Subtask 2.1
- [ ] **Documentation Sync** (MANDATORY)
  - [ ] Update package README.md with new nodes/services/actions/topics
  - [ ] Add docstrings to new Python modules/classes
  - [ ] If LAST STORY in epic: Update epic tech-spec with Implementation Status section
  - [ ] If LAST STORY in epic: Update epics.md Implementation Summary for this epic

## Dev Notes

- Relevant architecture patterns and constraints
- Source tree components to touch
- Testing standards summary

### Project Structure Notes

- Alignment with unified project structure (paths, modules, naming)
- Detected conflicts or variances (with rationale)

### Documentation Sync Requirements

**Per-Story (REQUIRED):**
- Update `ros2_ws/src/manipulator_control/README.md` with new functionality
- Add docstrings to all new Python modules/classes/functions

**If Last Story in Epic (REQUIRED):**
- Add "Implementation Status" section to `docs/sprint-artifacts/tech-spec-epic-{N}.md`
- Update "Implementation Summary" in `docs/epics.md` for this epic
- See `docs/epics.md#Epic-Level Documentation Sync` for template

### References

- Cite all technical details with source paths and sections, e.g. [Source: docs/<file>.md#Section]

## Dev Agent Record

### Context Reference

<!-- Path(s) to story context XML will be added here by context workflow -->

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List
