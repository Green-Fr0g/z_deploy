"""multi-robot MuJoCo simulation environment with ROS 2 interfaces.

Modules:
    config  -- configuration loading and path resolution
    model   -- MuJoCo model wrapper (actuators, state, reset)
    node    -- ROS 2 publishers, subscriber and service
    viewer  -- passive viewer camera behaviour
    app     -- simulation loop, signals and shutdown
    cli     -- command line entry point
"""
