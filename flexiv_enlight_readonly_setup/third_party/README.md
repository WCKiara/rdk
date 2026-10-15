# Official upstream source

The clone is at `flexiv_rdk/`, downloaded from https://github.com/flexivrobotics/flexiv_rdk. The clone is ignored so it is not accidentally vendored. Full commit and SHA-256 inspection evidence for the reviewed state-display example are in `../docs/source_review.json`.

At the user's explicit request on 2026-10-13, `example_py/intermediate4_non_realtime_cartesian_motion_force_control.py` was locally customized with Chinese parameter comments, single-arm selection (default ARM_1), simultaneous diagonal position motion and tool orientation changes, and monitored TCP position bounds. It uses pure position control. `--home-only` executes just the selected arm's preset Home primitive with boundary monitoring, then stops; diagonal motion is a separate invocation. The default Z lower bound is the fixed midpoint of the measured post-Home Z and the pre-Home Z. These are software bounds, not controller safety settings. The clone's original version can be retrieved from its Git history.

The local `--return-home` option adds a monitored return of the selected arm to preset Home after successful Cartesian arrival; interruption, timeout or failure stops without starting Home. The combined sequence remains unexecuted on hardware.

The direct NRT motion method now sends one final target instead of streamed intermediate targets. Optional `--motion-method movel` sends a single non-real-time MoveL primitive with the official Enlight 3E.1 parameter units. Both paths monitor position bounds and keep the return-Home option; the updated paths remain unverified for hardware trajectory feasibility.

To obtain the original source when online:

```bash
git clone https://github.com/flexivrobotics/flexiv_rdk.git third_party/flexiv_rdk
git -C third_party/flexiv_rdk checkout 159d41f3ab7fbd74117b6357ba612a0842bed6b9
```

Read the clone's README and `llms.txt`. The latter describes actuation patterns for general control projects; read-only diagnostics must continue to avoid those patterns. The separately requested motion customization above is outside the initial read-only scope.

The official `example_py/basics1_display_robot_states.py` exists but invokes `ClearFault()` and `ServoOn()`. It is not approved for hardware execution as a read-only diagnostic and remains unchanged. A separate proposed alternative is clearly labeled in `../scripts/proposed_readonly_states.py`; operator inspection and explicit approval are required before using it.
