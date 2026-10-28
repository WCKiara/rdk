# Flexiv Enlight read-only setup

Originally prepared for state observation with the official Flexiv RDK. This project does not use ROS, cameras, grippers, hands, or robot-learning software.

Whole-arm contact estimates can be read and recorded with `scripts/record_whole_arm_contact.py`; see [the data and usage notes](docs/WHOLE_ARM_CONTACT.md).

## RDK v2.1 installation

The official RDK v2.1 source is at `third_party/flexiv_rdk`, pinned to commit `159d41f3ab7fbd74117b6357ba612a0842bed6b9` (`v2.1-1-g159d41f`, see [the inspection record](docs/source_review.json)) with no local changes. The local C++ runtime is installed at `.rdk-v2.1`, using the SHA256-verified official v2.1 static archive and the installed Python 2.1.0 wheel's runtime dependencies (see `.rdk-v2.1/PROVENANCE.txt`). This keeps C++ programs and the Python `.venv` on the same SDK version. Configure a C++ consumer from the project root with:

```bash
cmake -S <YOUR_CPP_PROJECT> -B <YOUR_BUILD_DIRECTORY> \
  -Dflexiv_rdk_DIR="$PWD/.rdk-v2.1/lib/cmake/flexiv_rdk"
```

The local offline link check never constructs a Robot or contacts hardware:

```bash
cmake -S .build/rdk-v2.1-smoke-src -B .build/rdk-v2.1-smoke -DCMAKE_PREFIX_PATH="$PWD/.rdk-v2.1"
cmake --build .build/rdk-v2.1-smoke --parallel 2
.build/rdk-v2.1-smoke/rdk21_offline_check
```

Observed environment: native Ubuntu 22.04.5, x86_64, Python 3.10.12. No VM was detected. Commands run by the assistant may have sandbox network restrictions; run the diagnostics in a normal terminal on the same computer where Python will connect.

The existing system `python3` imports RDK **1.7.0**; this project's isolated `.venv` imports **2.1.0**. Global software was not changed. Select the RDK only after verifying actual controller software in Elements: the fetched official compatibility table pairs RDK 1.7 with robot software 3.9. Compatibility of project 2.1.0 with this controller remains unknown. The proposed alternative below targets v2 joint-group APIs and must not be used with global v1.7. Both reviewed upstream v1.7 and current examples enable the robot and are blocked under the read-only scope.

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
