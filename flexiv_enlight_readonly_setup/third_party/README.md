# Official upstream source

The official source from https://github.com/flexivrobotics/flexiv_rdk is included at `flexiv_rdk/`, checked out at the reviewed v2.1 commit with no local changes.

To obtain the same source when online:

```bash
git clone https://github.com/flexivrobotics/flexiv_rdk.git third_party/flexiv_rdk
git -C third_party/flexiv_rdk checkout 159d41f3ab7fbd74117b6357ba612a0842bed6b9
```

Read the clone's README and `llms.txt`. The latter describes actuation patterns for general control projects; read-only diagnostics must continue to avoid those patterns.

The official `example_py/basics1_display_robot_states.py` exists but invokes `ClearFault()` and `ServoOn()`. It is not approved for hardware execution as a read-only diagnostic and remains unchanged. A separate proposed alternative is clearly labeled in `../scripts/proposed_readonly_states.py`; operator inspection and explicit approval are required before using it.
