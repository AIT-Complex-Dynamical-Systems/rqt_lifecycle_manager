# rqt_lifecycle_manager

A rqt plugin showing a list of all discovered lifecycle nodes and their current state.

From the right-click context menu all transitions from the current state can be triggered according to the [lifecycle node design document](https://design.ros2.org/articles/node_lifecycle.html). It does not check for actual available transitions.

## Parameters

The plugin reads the following ROS parameters from the rqt node:

- `rqt_lifecycle_manager.update_freq` (float, default: `1.0`) - update rate in Hz for node list and state refresh.
- `rqt_lifecycle_manager.service_timeout` (float, default: `1.0`) - timeout in seconds for lifecycle service availability and responses.

All values must be strictly greater than `0`. Invalid values are ignored and the defaults are used.
