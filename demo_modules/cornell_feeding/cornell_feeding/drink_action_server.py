"""
cornell_feeding drink action server — thin shim to RAMMP's implementation.

The real Cornell drink node is `rammp.integration.drink_action_server` from
empriselab/RAMMP (feature/improve-performance), vendored into this package at
``cornell_feeding/rammp`` so it ships self-contained. It is written against
`cornell_feeding_interfaces` (DrinkAction / CupInfo), runs as the `drink_action_server`
node, exposes the /arm/drink/* action servers + the /arm/drink/detection/enable
streaming service, and drives the shared arm_driver via /arm/* as the `cornell` source.

This module re-exports the vendored ``main`` so the node ships as the `cornell_feeding`
executable (``ros2 run cornell_feeding drink_action_server``).
"""

from rammp.integration.drink_action_server import main

if __name__ == "__main__":
    main()
