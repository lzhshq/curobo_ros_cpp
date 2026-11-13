# Collision world

`PlannerCore` owns the shared `Scene`, the object registry, and the
`MotionPlanner`. ROS exposes that world through `curobo_planner` services.

## Services

Namespace: `/curobo_planner`.

| Service | Type | Purpose |
|---|---|---|
| `add_object` | `curobo_planner_interfaces/AddObject` | Add a cuboid, sphere, capsule or cylinder |
| `update_object_pose` | `curobo_planner_interfaces/UpdateObjectPose` | Move an existing object without rebuilding the planner |
| `remove_object` | `curobo_planner_interfaces/RemoveObject` | Remove one object |
| `clear_objects` | `curobo_planner_interfaces/ClearObjects` | Remove every object |
| `list_objects` | `curobo_planner_interfaces/ListObjects` | List object names |

`update_object_pose` is the dynamic-obstacle path: `PlannerCore` calls
`Scene::update_obstacle_pose` for the stored handle, so CUDA storage is reused.

## Example

```bash
ros2 service call /curobo_planner/add_object \
  curobo_planner_interfaces/srv/AddObject \
  "{type: 2, name: bat, pose: {position: {x: 0.3, y: 0.0, z: 0.3}, orientation: {w: 1.0}}, dimensions: {x: 0.03, y: 0.4, z: 0.0}}"

ros2 service call /curobo_planner/update_object_pose \
  curobo_planner_interfaces/srv/UpdateObjectPose \
  "{name: bat, pose: {position: {x: 0.3, y: 0.0, z: 0.45}, orientation: {w: 1.0}}}"

ros2 service call /curobo_planner/list_objects \
  curobo_planner_interfaces/srv/ListObjects "{}"
```

## Type semantics

```text
CUBOID:  dimensions = full extents [dx, dy, dz]
SPHERE:  dimensions.x = radius
CAPSULE: dimensions.x = radius, dimensions.y = length (local Z)
CYLINDER:dimensions.x = radius, dimensions.y = height
```

Capsules and cylinders are local +Z; rotate the object pose to align them with
another world axis.

## Current boundary

The `curobo_planner` services own the planner world. The MPC node still creates
its own `PlannerCore` scene; a later step should make the MPC consume the same
world (service/topic or one unified node). The API and update semantics are
already shared so that migration does not change the obstacle model.


## Generic update topic

Planner and MPC both subscribe to:

```text
/curobo_world/obstacle_update
```

Type:

```text
curobo_planner_interfaces/ObstacleUpdate
```

Commands:

```text
ADD / UPDATE / REMOVE / CLEAR
```

The publisher uses transient-local QoS. A late subscriber that only receives
`UPDATE` recreates the object from the update payload, so startup order does not
matter.
