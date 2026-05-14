# amr_description

AMR-owned robot description package.

## Responsibility

- own the TurtleBot3 Burger-compatible URDF/xacro used by the AMR stack
- keep robot-side frame names stable for navigation, RViz, and `amr_visualization`
- remove the long-term runtime need for `turtlebot3_description`

The first implementation pass adds a minimal TurtleBot3 Burger-compatible description that can be refined later.
