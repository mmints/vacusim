# VacuSim Planning Node: Technical Overview

This document explains what the planning package does, how to run it, how to call its service, and how to use the returned path.

## 1) What this package provides

Package: `vacusim_robot_planning`

Main node: `planner_node`

Main responsibility:
- Accept a start pose and a goal pose (in tile coordinates)
- Plan a collision-safe route on the map
- Return route waypoints as `geometry_msgs/Pose2D[]`

The planner avoids map obstacles:
- `0` = wall
- `9` = furniture

It also applies a robot safety buffer (`robot_buffer`, default `0.5` tile), so routes keep extra clearance from obstacles.

## 2) Inputs to the planner

The planner reads:
- `map_path` (string): path to map file
- `service_name` (string): service topic name, default `plan_path`
- `robot_buffer` (float): clearance around obstacles, default `0.5`

Map format:
- Text grid of digits
- One row per line
- Cells with `1..8` are traversable
- Cells with `0` or `9` are blocked

## 3) How to start the planner

### Option A: launch file (recommended)

```bash
ros2 launch vacusim_robot_planning planner_launch.py
```

With custom map:

```bash
ros2 launch vacusim_robot_planning planner_launch.py map_path:=/absolute/path/to/map.txt
```

### Option B: run node directly (for custom parameters)

```bash
ros2 run vacusim_robot_planning planner_node --ros-args -p map_path:=/absolute/path/to/map.txt -p service_name:=plan_path -p robot_buffer:=0.5
```

## 4) Service interface

Service type:

`vacusim_robot_interfaces/srv/PlanPath`

Request:
- `geometry_msgs/Pose2D start`
- `geometry_msgs/Pose2D goal`

Response:
- `bool success`
- `string message`
- `geometry_msgs/Pose2D[] waypoints`

Notes:
- `theta` in request can be set to `0.0` (planner uses position for planning).
- Response waypoints are in tile coordinates.
- Planner may accept replanning from a currently occupied start cell, but still avoids obstacles for the produced path.

## 5) How to call the service from terminal

First verify service exists:

```bash
ros2 service list | grep plan_path
```

Call example:

```bash
ros2 service call /plan_path vacusim_robot_interfaces/srv/PlanPath "{start: {x: 2.0, y: 2.0, theta: 0.0}, goal: {x: 16.0, y: 19.0, theta: 0.0}}"
```

Expected response structure:

```text
success: true
message: Planned N waypoint(s).
waypoints:
- x: ...
  y: ...
  theta: ...
- ...
```

If planning fails:

```text
success: false
message: <reason>
waypoints: []
```

## 6) What to do with planner output

Use returned waypoints as intermediate navigation targets:
1. Take waypoint `0` as current target
2. Turn robot toward waypoint
3. Drive toward waypoint
4. When within threshold, switch to next waypoint
5. Repeat until final waypoint

Recommended controller behavior:
- Use a waypoint reach threshold (for example `0.5` tile)
- Replan if robot deviates significantly or gets blocked
- Keep obstacle avoidance active while executing waypoints

## 7) Example from the current room controller

The controller already demonstrates service usage:
- Creates client for `PlanPath`
- Sends current pose as `start`
- Sends destination as `goal`
- On `success == true`, stores `response.waypoints`
- Follows waypoints in sequence

Implementation reference:
- `_plan_path()` in `vacusim_room_controller/vacusim_room_controller/room_controller.py`

## 8) Common troubleshooting

Service not found:
- Ensure planner node is running
- Check service name (`/plan_path` by default)

Planning fails immediately:
- Verify start/goal are inside map bounds
- Verify goal is not inside wall/furniture
- If `robot_buffer` is too large, narrow passages may become unavailable

Robot path too close to obstacles:
- Increase `robot_buffer` (for example `0.6` or `0.7`)

No waypoints but success false:
- Check `message` field; it includes reason (bounds, occupied goal, or no path)
