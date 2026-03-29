---
name: nav2_navigation
description: Send navigation goals to Nav2, query goal status, and cancel active goals.
---

# Nav2 Navigation

Use this skill to control Nav2 through the `nav2-rbnx` package.

## Tools

- `navigate_to`
- `get_nav_status`
- `cancel_nav`

## Guidance

- Use `navigate_to` with map-frame coordinates.
- Use `get_nav_status` to monitor progress after dispatching a goal.
- Use `cancel_nav` if the robot should stop pursuing an active goal.
