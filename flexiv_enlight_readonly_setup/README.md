# Flexiv Enlight read-only setup

Originally prepared for state observation with the official Flexiv RDK. This project does not use ROS, cameras, grippers, hands, or robot-learning software.

Whole-arm contact estimates can be read and recorded with `scripts/record_whole_arm_contact.py`; see [the data and usage notes](docs/WHOLE_ARM_CONTACT.md).

## Current intermediate6: ARM_1 polishing

As of 2026-10-21, the current `third_party/flexiv_rdk/example/intermediate6_realtime_cartesian_motion_force_control.cpp` explicitly addresses only `ARM_1` for startup Home, the 15-degree WORLD +Y tilt, F/T zeroing, WORLD -Z contact search, per-arm force settings and real-time targets. It accepts an Enlight dual-arm robot when `ARM_1` exists. An absent `ARM_1` is rejected before control requests; `--arm ARM_2`, other arm values and unknown arguments are rejected before connection. Zeroing waits for `ARM_1`'s own completion state.

The robot must start fault-free, IDLE and all-stopped. By default it must already be operational. `--auto-servo-on` explicitly permits one robot-wide `ServoOn()` from `NOT_SERVO_ON`, with a 10-second readiness deadline; this may release both arms' brakes. Mode switching and Stop are also robot-wide SDK operations. Sending targets only to `ARM_1` does not mechanically lock `ARM_2`.

The operator's 2026-10-21 log shows a controller rejection of the initial non-real-time tilt target before any observed tilt motion. It does not establish an unreachable pose, a joint-limit violation, or a quaternion-order error. The near-180-degree measured Home quaternion and a negative target `qw` are valid representations; feasibility remains a controller-side question.

`--tilt-method nrt` retains the original direct non-real-time tilt command and is the default. Optional `--tilt-method movel` sends one ARM_1 `MoveL` primitive for the same fixed-position, 15-degree WORLD +Y target. MoveL receives WORLD position in metres, Euler ZYX orientation and measured ARM_1 reference joints in degrees, angular velocity converted from 0.10 rad/s to degrees/s, `vel=0.01`, `acc=0.10` and `zoneRadius=ZFine`. These units and parameter ranges follow the [Enlight MoveL manual](https://www.flexiv.com/software/primitives/en/3E.1/enlight/Motion.html#movel). Neither method automatically retries, switches planners after rejection, or clears a fault. MoveL has not been verified to make the reported target feasible.

`--tilt-only` runs ARM_1 Home and the selected initial tilt, then calls Stop and exits before F/T zeroing, contact search or RT control. For an operator-chosen isolated tilt trial after checking the controller's fault and stopped state:

```bash
sudo third_party/flexiv_rdk/example/build/intermediate6_realtime_cartesian_motion_force_control \
  '<ACTUAL_FULL_SERIAL>' --arm ARM_1 --tilt-method movel --tilt-only
```

Tilt planning prints full-precision `TILT_START`, `TILT_TARGET`, relative angle and starting ARM_1 joints. On a control exception, Stop is attempted first; the program then prints the phase, stop-request outcome, measured state and the same Robot instance's event IDs, descriptions, causes and recommended actions. A rejected Stop request is reported as unconfirmed, and does not replace the original exception. SDK event history is limited to the current connection lifetime. The pre-change source and binary are preserved in `.build/before-tilt-trajectory-fix-20261021/`.

Build the current target from this project directory:

```bash
cmake -S third_party/flexiv_rdk/example -B third_party/flexiv_rdk/example/build \
  -DCMAKE_PREFIX_PATH="$HOME/rdk_install"
cmake --build third_party/flexiv_rdk/example/build \
  --target intermediate6_realtime_cartesian_motion_force_control --parallel 4
```

The following command starts the ARM_1 Home/tilt/zero/contact-search workflow and then the 5 cm WORLD +Y real-time line with Z force control. Omitting `--polish` retains Home, tilt and contact search, then holds the contact pose instead of executing the line.

```bash
sudo third_party/flexiv_rdk/example/build/intermediate6_realtime_cartesian_motion_force_control \
  '<ACTUAL_FULL_SERIAL>' --arm ARM_1 --polish
```

Source and binary from before this change are preserved in `.build/before-arm1-only-20261021/`. Verification uses a real-SDK build and offline in-memory Robot/Scheduler replacements; this change has not been run against a robot. The older C++ interfaces and commands described below are historical and apply to earlier implementations.

## Earlier setup and motion notes

On 2026-10-13 the user separately requested a motion customization of `third_party/flexiv_rdk/example_py/intermediate4_non_realtime_cartesian_motion_force_control.py`. This local version defaults to ARM_1, supports `--delta`/`--position` and `--tilt`/`--rpy`, checks finite software TCP position bounds, and stops on completion, interruption, timeout or an observed boundary violation. It does not automatically home or search for contact. `--home-only` performs only the selected arm's preset Home primitive. Home is a joint trajectory monitored online; these TCP bounds do not guarantee the whole arm's path or replace controller safety limits. On the user's explicit instruction, ARM_1-only Home completed and IDLE/all-stopped were verified. Diagonal/tilting motion has not been executed on hardware.

The Z lower bound was then fixed at the midpoint of post-Home measured Z and pre-Home Z: `0.04727903753519058 m`. X bounds are `[0.35, 0.46]`, Y bounds `[0.15, 0.25]`, and Z upper bound `0.18 m`; all are in WORLD. A `0.005 m` inner margin causes the position monitor to stop at the inner guard. Override bounds with `--position-min`/`--position-max`. `--preview` reads the current pose and prints the checked plan without servo, mode or motion commands. Use the project `.venv`; scipy is now included in the dependency files. The read-only diagnostic restrictions below remain applicable to those diagnostics; the user's specific motion instructions authorize the customized example separately.

Add `--return-home` to a `--delta`/`--position` invocation to stop after successful arrival, return the same arm to its preset Home while monitoring the same bounds, and stop again. It never starts Home after interruption, failure, timeout or a boundary violation. The combined movement/return sequence has been tested offline and has not been executed on hardware.

Startup diagnostics distinguish a disconnected transport from a reported fault and include `operational_status()`. Optional `--clear-minor-fault` attempts one startup recovery only for `MINOR_FAULT` while IDLE and all-stopped, with `--fault-timeout` (default 5 seconds); critical faults and faults during movement are not cleared by this path. This option cannot be combined with read-only `--preview`. A reported startup MINOR_FAULT on 2026-10-13 was cleared with a single SDK request, followed by repeated connected/no-fault/READY/IDLE/all-stopped checks. No servo, Home or movement commands were sent during that recovery.

Following a user-reported NRT trajectory generation failure, direct `--motion-method nrt` now sends the final target exactly once and monitors arrival. It no longer sends interpolated micro-targets each cycle. `--motion-method movel` uses the non-real-time MoveL primitive in NRT_PRIMITIVE_EXECUTION, also sent exactly once. MoveL receives `Coord` position in metres and orientation in degrees, `vel` in m/s, `angVel` converted from the CLI rad/s to deg/s, `--movel-acc` in m/s² (default 0.1), and `zoneRadius=ZFine`. Its contact limit is a software measured-wrench check; SetMaxContactWrench is not applicable in that primitive mode. `frequency` controls monitoring; `duration` is an estimate/timeout budget, not a guaranteed motion duration. Both methods keep the configured position bounds and optional return Home. The updated motion methods have passed offline tests and installed-SDK data construction checks; hardware trajectory feasibility has not been verified.

## ARM_1 real-time end-effector motion with contact recording

The locally customized C++ `example/intermediate6_realtime_cartesian_motion_force_control.cpp` now streams only ARM_1 end-effector poses at 1000 Hz. It keeps WORLD `--delta`, tool-relative `--tilt` in degrees, the same TCP position bounds and 5 mm inner margin, and optional successful-arrival-only ARM_1 Home. All six Cartesian axes use motion control; the original double-arm contact search and pressing-force demonstration have been removed. `--arm ARM_2` is rejected before connection. The retained positional `50` controls Home status polling, not streaming frequency. Replace NRT `--movel-acc` with RT `--max-linear-acc`; omit `--motion-method movel`. A quintic trajectory extends `--duration` when needed to respect both translational and angular velocity/acceleration limits.

`scripts/run_realtime_cartesian_with_contact.py` waits for the existing ARM_1 recorder to produce a valid CSV row before starting the motion binary. It stops motion if recording exits and cleans up both processes on interruption. CSV paths default to unique UTC timestamps in `logs/`; these contain per-link contact force/position estimates, not tactile skin arrays. The recorder runs as the original sudo user; the C++ child retains root scheduler privileges. Only motion completion starts return Home; failures, interruptions and observed boundary/ARM_2 violations stop without Home.

The local RDK 2.1 runtime is installed at `.rdk-v2.1`, using the SHA256-verified official v2.1 static archive and the installed Python 2.1.0 wheel's runtime dependencies (see `.rdk-v2.1/PROVENANCE.txt`). This keeps the C++ motion and Python recorder on the same SDK version. Build only this target from the project root:

```bash
cmake -S third_party/flexiv_rdk/example -B .build/realtime-examples \
  -Dflexiv_rdk_DIR="$PWD/.rdk-v2.1/lib/cmake/flexiv_rdk"
cmake --build .build/realtime-examples \
  --target intermediate6_realtime_cartesian_motion_force_control --parallel 2
```

The requested combined command is:

```bash
sudo .venv/bin/python scripts/run_realtime_cartesian_with_contact.py \
  '<ACTUAL_FULL_SERIAL>' 50 \
  --arm ARM_1 \
  --delta 0.01 0 -0.01 \
  --tilt 0 15 0 \
  --max-linear-vel 0.01 \
  --max-angular-vel 0.15 \
  --max-linear-acc 0.1 \
  --duration 5 \
  --position-min 0.35 0.15 0.04727903753519058 \
  --position-max 0.46 0.25 0.18 \
  --clear-minor-fault \
  --auto-servo-on \
  --return-home \
  --record-contact \
  --contact-rate 20
```

Add `--auto-servo-on` to either the binary or launcher command to enable startup servo recovery from `NOT_SERVO_ON`. This is one robot-wide `ServoOn()` request and may release both arms' brakes, even though motion and Home targets address ARM_1 only. The robot must be IDLE/all-stopped, the initial trajectory must pass validation, and both arms must provide fresh states before the request. It waits for operational READY within `--startup-timeout` (default 10 seconds), monitors the existing TCP/wrench and ARM_2 guards during the wait, then rebuilds the trajectory from the measured post-enable ARM_1 pose. The ARM_2 baseline is retained across enable. Already-READY robots skip ServoOn; an ongoing brake release is waited on without a second request. Other non-ready statuses are rejected. Failure or interruption after a ServoOn attempt calls Stop without starting Home; no runtime fault recovery or ServoOn retry is performed. Without the flag, the robot must already be operational.

For a read-only pose/plan preview, omit `sudo` and `--clear-minor-fault`, and add `--preview`. Preview does not start recording or send control commands, including when `--auto-servo-on` is present. The source had been overwritten with duplicated official-example code; the customized ARM_1 implementation was reconstructed using the retained trajectory helpers, documented interface and lifecycle tests before adding auto-enable. The previous source and binary are backed up in `.build/before-auto-servo-on/`. Build/help checks, numerical trajectory tests and fake-SDK lifecycle tests cover the implementation; automatic enable and the new real-time motion have not been executed on hardware by this change.

Every streamed command and Home primitive contains only ARM_1. ARM_2's seven joint positions are checked against the startup baseline; measured drift above 0.001 rad aborts immediately. Its 0.01 rad/s speed check now uses average position changes over a trailing 100 ms window, with a fixed-size history buffer. Raw SDK `dq` must have seven finite values but no longer triggers a stop on a single noisy value. Startup requires at least 100 ms of fresh ARM_2 position history before control; both arms' timestamps must advance and remain fresh. Oscillations within the observation window may cancel in the average speed; the baseline position check therefore applies independently on every sample, including warmup and servo enable. This is a software observation and stop, not a mechanical lock. Robot-wide ServoOn is used only with explicit `--auto-servo-on`; mode switching and Stop are also robot-wide SDK operations. The software TCP box does not bound the whole arm/tool geometry; the controller's Home is a joint trajectory with online TCP monitoring rather than a prevalidated Cartesian path.

The real-time scheduler is now constructed before the Robot interface and started with output gated while the robot is still IDLE. Startup uses `--startup-timeout` to wait for its first callback; only after `RT_SCHEDULER_READY` does the program enter RT mode, configure ARM_1, verify the captured start pose and enable trajectory output. The installed SDK has seconds of blocking scheduler initialization/startup, so the old 20 ms heartbeat check could reject startup before the first target. The 20 ms runtime callback watchdog remains active once callbacks are running, and the motion/arrival budget starts when trajectory output is enabled. Callback failures retain their specific errors after scheduler threads are joined. `RT_ABORT` reports phase, callbacks, attempted stream calls (`command_attempts`), successfully returned calls (`commands`) and heartbeat age at shutdown request. A zero attempt count confirms no streaming call was made; returned calls alone do not prove physical trajectory execution. No ARM_2 hold or trajectory target was added; its existing measured-position guards remain active during monitoring. The command line is unchanged. The pre-fix source/binary are backed up in `.build/before-rt-startup-fix/`; verification uses fake-SDK delay/stall regression cases and the real SDK build, without robot actuation.

For a trajectory starting at the operator-confirmed current Home, the program captures ARM_1's measured world TCP position and quaternion after servo readiness while still IDLE. This is the fixed trajectory origin; the first streamed target holds that same pose. It does not execute Home on startup or substitute a nominal identity orientation. The origin is not silently redefined after scheduler startup or RT mode configuration. Before the first target it checks that the current pose remains within 1 mm and 1 degree of the capture, matching the existing arrival tolerances; otherwise it stops without sending a trajectory target. `--delta` is added in WORLD and `--tilt` rotates relative to the captured initial tool frame. Preview prints both start and target, plus their relative angle. The software accepts the operator's Home assertion; it does not independently verify the controller's preset Home joint configuration.

The reported `orientation error exceeds 90 degree` occurred after four returned stream calls, before arrival or Home. Reconstructing the start from the supplied target gives a world orientation near 180 degrees about Y, while the first four milliseconds of the requested 5-second trajectory rotate by less than 0.000001 degree. The quaternion order matches the v2.1 API (`qw qx qy qz`), so the provided log does not establish a 15-degree trajectory-angle error. The controller-side cause remains unconfirmed. Failures now print a fixed-size last-eight-frame trace of measured/sent ARM_1 poses, both arms' measured poses and controller desired poses, and the same Robot instance's `event_log()` event IDs, descriptions and causes. After scheduler threads are joined, nonblocking accessors capture the pre-Stop poses; the program attempts Stop before printing any failure trace or events. Diagnostic output sends no control requests; event history from earlier connections is unavailable. The pre-change source/binary are in `.build/before-home-start-check/`. Near-180-degree Home quaternion and startup-motion regressions are tested offline; no hardware trajectory was run to prepare this change.

The earlier `ARM_2 movement detected` startup rejection used raw `dq` with a single-sample 0.01 rad/s threshold. The operator's 2026-10-20 read-only samples show a maximum ARM_2 position range of 0.0000053048 rad over about two seconds, while raw `dq` reaches 0.0827585 rad/s. The SDK documents link-encoder `dq` as a noisy measurement. This motivated the position-window speed check; the baseline position threshold is retained. These sparse position samples alone do not prove absence of all motion between samples. The same read-only diagnostic verified live state timestamps and robot-reported software v4.1; no motion was run during this investigation.

If the ARM_2 stationary check rejects startup, it does not mean a target was sent to ARM_2. Diagnostics report the phase, one-based joint number, signed position change and `speed_100ms` with their thresholds and `speed_source=position_window_100ms`. Invalid raw `dq` reports the joint and startup/runtime phase separately. An error before the first `ARM_1 only` plan line occurs before motion mode or targets are sent. Inspect state data without repeating movement:

```bash
.venv/bin/python scripts/connect_readonly.py \
  --serial '<ACTUAL_FULL_SERIAL>' --connect-readonly --timeout-seconds 15
```

The two SDK connection messages during a combined invocation come from the recorder and the motion program, rather than one connection per arm.

Observed environment: native Ubuntu 22.04.5, x86_64, Python 3.10.12. No VM was detected. Commands run by the assistant may have sandbox network restrictions; run the diagnostics in a normal terminal on the same computer where Python will connect.

The existing system `python3` imports RDK **1.7.0**; this project's isolated `.venv` imports **2.1.0**. Global software was not changed. Select the RDK only after verifying actual controller software in Elements: the fetched official compatibility table pairs RDK 1.7 with robot software 3.9. Compatibility of project 2.1.0 with this controller remains unknown. The proposed alternative below targets v2 joint-group APIs and must not be used with global v1.7. Both reviewed upstream v1.7 and current examples enable the robot and are blocked under the read-only scope.

At the user's subsequent request, the official **C++ SDK v2.2 draft** was installed locally at `.rdk-v2.2` and passed offline header/link/runtime verification. **Python v2.2 is not published** on PyPI or as a release wheel; existing Python packages are unchanged. See [v2.2 installation evidence and commands](docs/RDK_V2_2_INSTALL.md).

## Critical finding

The reviewed official `example_py/basics1_display_robot_states.py` is **not strictly read-only**: it clears faults and turns servos on. The unchanged source remains in `third_party/flexiv_rdk`, but the guarded wrapper refuses it. See [the inspection record](docs/source_review.json) and [setup report](docs/SETUP_REPORT.md). Do not run the raw official example under this project's restrictions.

`scripts/proposed_readonly_states.py` is a separate proposal using only robot connection and info/state accessors. It is not an upstream example and has not been run against hardware. An operator must inspect this exact file and specifically approve it before it can be used. It refuses missing serials and omitted `--connect-readonly`; a parent process bounds the connection/display time.

The subsequent `scripts/connect_readonly.py` diagnostic targets exactly RDK 2.1.0 and reads the actual controller's reported arm groups, accommodating the operator's Enlight-LL target. It received operator approval; the operator has now reported successful dual-arm live-state verification with `LIVE_STATES_VERIFIED`. On 2026-10-13, a separate live read-only request/reply test also connected successfully and received three controller replies. Use the project `.venv`; system Python still imports 1.7.0. [The official v2.1 release notes](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v2.1) specify robot software **3E.1**, confirmed by the operator. Actual identifiers are stored only in ignored local configuration/report files.

After code review and operator confirmation, its invocation is:

```bash
.venv/bin/python scripts/connect_readonly.py --serial '<ACTUAL_FULL_SERIAL>' --connect-readonly --timeout-seconds 15
```

This uses only connection/info/state accessors and never performs the servo or fault operations found in the official example. Omitted hardware opt-in is refused. Do not declare live-state success unless it prints `LIVE_STATES_VERIFIED` after finite samples and advancing timestamps.

## Read-only request/reply test

`scripts/test_request_reply.py` sends three blocking `global_variables()` queries and waits for controller replies. It prints only reply counts and elapsed times; zero variables is a valid reply. The observed test passed all three requests in 0.210–0.299 ms and printed `REQUEST_REPLY_VERIFIED`. It sends no motion, enable, mode-switch, or write commands. This verifies read-only request/reply communication; motion command delivery remains untested.

With the actual serial in `FLEXIV_ROBOT_SERIAL`, run:

```bash
.venv/bin/python scripts/test_request_reply.py --connect-readonly --timeout-seconds 20
```

The parent process bounds the test; timeout or interruption is a failure/cancellation, never success. See [the latest evidence](docs/SETUP_REPORT.md).

## Safety and manual prerequisites

For read-only diagnostics, never issue enable/disable, fault clearing, mode switching, homing, free-drive, motion, force, tool, or trajectory commands. Preserve interlocks and security settings. Robot power, switches, cabling, Elements, controller licensing/settings, and Auto (Remote) are manual actions for an authorized operator following the supplied robot manual and lab rules. No such local manual was available during preparation.

Complete [the hardware checklist](docs/MANUAL_HARDWARE_CHECKLIST.md), confirm the real serial and IP, inspect the approved diagnostic, and approve the exact command before hardware contact. Ping alone does not verify an RDK session. Live verification requires nonempty finite joint samples and changing timestamps from the actual robot.

## Fast path — Ubuntu 22.04+ with supported 64-bit Python 3.10

Run from this project directory. Existing `.venv` and upstream clone have already been created here; skip recreation on this machine.

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
git clone https://github.com/flexivrobotics/flexiv_rdk.git third_party/flexiv_rdk
python scripts/check_environment.py
python scripts/check_rdk_import.py
python -m compileall -q scripts tests
python -m unittest discover -s tests -v
python scripts/network_diagnostics.py
```

For the recorded dependency versions, install `requirements.lock.txt` instead of `requirements.txt`. Pin the clone to the commit in `docs/source_review.json`; changing upstream source requires re-inspection, not merely updating its hash.

After the operator verifies the actual IP:

```bash
python scripts/network_diagnostics.py --robot-ip '<VERIFIED_ROBOT_IP>' --timeout 2
```

Network/OS changes require an explicit, reviewed proposal and approval. Never reconfigure the remote-session interface. See [network setup](docs/NETWORK_SETUP.md).

## Windows PowerShell

Windows 10+ x86_64 with supported Python 3.10 or 3.12 is documented upstream; Windows was not tested here. These commands use Python 3.10.

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
git clone https://github.com/flexivrobotics/flexiv_rdk.git third_party\flexiv_rdk
python scripts\check_environment.py
python scripts\check_rdk_import.py
python -m unittest discover -s tests -v
python scripts\network_diagnostics.py
```

Do not change system execution policy automatically if activation is blocked; invoke `.\.venv\Scripts\python.exe` directly instead. Do not assume a Linux-generated lock guarantees Windows wheel availability.

## Network choices and VMs

Direct User Port 1 uses a static PC address in the operator-verified controller subnet. Factory defaults are robot `192.168.2.100/24`, with an illustrative PC `192.168.2.10/24`, no gateway. User Port 2 uses a shared DHCP router. General Port serves DHCP to a directly connected PC; its documented default is `192.168.100.1`. Treat all defaults as examples until verified.

For an Ubuntu VM, NAT can impede local robot discovery. Ask the operator to configure bridging to the wired NIC or pass through a USB Ethernet adapter under lab policy. Check routes and connectivity from both the Windows host and Ubuntu guest; host success does not establish guest connectivity. Bridging is not guaranteed to forward discovery traffic.

## Official invocation and guarded wrapper

Upstream's documented invocation is shown for identification only. **Do not execute it with the reviewed actuation-containing version.**

Ubuntu reference command:

```bash
cd third_party/flexiv_rdk/example_py
python basics1_display_robot_states.py '<ACTUAL_ROBOT_SERIAL>'
```

Windows reference command:

```powershell
cd third_party\flexiv_rdk\example_py
python .\basics1_display_robot_states.py '<ACTUAL_ROBOT_SERIAL>'
```

The wrapper invocation below is also blocked by the current source review. The wrapper checks exact path, SHA-256, commit, manual approval, API calls, venv, real serial, and hardware opt-in before launching the unchanged source.

```bash
python scripts/run_official_state_example.py --serial '<ACTUAL_ROBOT_SERIAL>' --connect-readonly --timeout-seconds 15
```

If the operator separately approves the inspected alternative, the reviewable command is:

```bash
python scripts/proposed_readonly_states.py --serial '<ACTUAL_ROBOT_SERIAL>' --connect-readonly --timeout-seconds 15
```

Ctrl+C stops the display process. The alternative prints a `LIVE_SAMPLE` marker only after it observes finite joint values and a changing timestamp; process exit alone is not live verification. Errors are printed and never corrected by sending robot commands. No logs are recorded automatically.

Put actual identifiers only in ignored `.env` or `config/robot.env`; these files are not automatically loaded. Set `FLEXIV_ROBOT_SERIAL` in the terminal environment or pass `--serial`. Do not publish commands/logs containing real serials. Example configuration contains placeholders only.

See [troubleshooting](docs/TROUBLESHOOTING.md), [setup evidence](docs/SETUP_REPORT.md), and [upstream provenance](third_party/README.md).
