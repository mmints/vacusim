# Slide Breakdown: How to Teach the VacuSim Planning Node

Use this as a ready outline for first-year college slides.

## Slide 1 - Why Planning?

Title: Why does a robot need a planner?

Key points:
- A direct straight line can hit walls or furniture
- The planner computes a safe route before movement
- The controller then follows that route step by step

Speaker line:
- "The planner decides where to go; the controller decides how to move there."

## Slide 2 - What is the Planning Package?

Title: What does `vacusim_robot_planning` do?

Key points:
- Runs as a ROS 2 node (`planner_node`)
- Reads a grid map
- Exposes a ROS 2 service: `/plan_path`
- Returns waypoints between start and goal

## Slide 3 - Map Meaning

Title: How the map is interpreted

Key points:
- `0` = wall (blocked)
- `9` = furniture (blocked)
- `1..8` = free tiles (can be used)
- Planner also uses a safety margin (`robot_buffer`)

Speaker line:
- "Not only obstacles are blocked; nearby cells can also be avoided for safety."

## Slide 4 - Service Contract

Title: Service input and output

Service type:
- `vacusim_robot_interfaces/srv/PlanPath`

Request:
- `start` (`Pose2D`)
- `goal` (`Pose2D`)

Response:
- `success` (`bool`)
- `message` (`string`)
- `waypoints` (`Pose2D[]`)

## Slide 5 - How to Run It

Title: Starting the planner node

Command:

```bash
ros2 launch vacusim_robot_planning planner_launch.py
```

Optional custom map:

```bash
ros2 launch vacusim_robot_planning planner_launch.py map_path:=/absolute/path/to/map.txt
```

## Slide 6 - How to Call the Service

Title: Terminal call example

```bash
ros2 service call /plan_path vacusim_robot_interfaces/srv/PlanPath "{start: {x: 2.0, y: 2.0, theta: 0.0}, goal: {x: 16.0, y: 19.0, theta: 0.0}}"
```

Explain:
- We ask: "Plan from start to goal"
- Planner replies with a list of waypoints

## Slide 7 - Reading the Response

Title: Understanding planner output

If success:
- `success: true`
- `message: Planned N waypoint(s).`
- `waypoints`: ordered targets to follow

If failure:
- `success: false`
- `message` explains why (out of bounds, occupied goal, no path)

## Slide 8 - What the Controller Does with Waypoints

Title: From plan to movement

Flow:
1. Pick next waypoint
2. Rotate toward it
3. Drive toward it
4. If close enough (threshold), switch to next waypoint
5. Repeat

Speaker line:
- "Waypoints are mini-goals that simplify complex navigation."

## Slide 9 - Real Project Example

Title: Where this is used in our project

Point students to:
- `vacusim_room_controller/vacusim_room_controller/room_controller.py`
- Method `_plan_path()`

What to mention:
- The controller calls `/plan_path`
- Stores `response.waypoints`
- Navigates waypoint by waypoint

## Slide 10 - Practical Tips and Limits

Title: Engineering tips

Tips:
- Keep waypoint tolerance (for example `0.5`) to avoid getting stuck
- Replan when environment changes
- Increase `robot_buffer` for safer clearance

Limits:
- If passages are narrow, large buffer may block all routes
- Planner gives a path, but safe execution still needs obstacle handling

## Slide 11 - Demo Plan

Title: Suggested classroom demo

Demo steps:
1. Start planner node
2. Call service with two points
3. Show returned waypoints
4. Start controller and observe waypoint following
5. Change goal and repeat

## Slide 12 - Recap

Title: Takeaway

Recap sentence:
- "The planner converts a map + start + goal into safe waypoints; the controller executes those waypoints to move the robot."
