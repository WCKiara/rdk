# Codex Instructions: Flexiv Enlight RDK First-Time Setup and Read-Only State Verification

## 0. Mission

You are Codex acting as a robotics software setup assistant. Prepare a reproducible, safe, **read-only** development environment for a **single Flexiv Enlight arm** using the official Flexiv Robotics Development Kit (RDK). The user is working in a laboratory and may be developing inside an **Ubuntu virtual machine** on a Windows host; discover the actual environment before choosing commands. Do not assume access to the physical robot or that the lab network has its factory defaults.

**Primary success criterion:** From the machine where Python runs, the user can invoke the official `example_py/basics1_display_robot_states.py` with the actual robot serial number and observe valid live robot state output. Do not claim physical verification unless the actual robot was contacted and the script produced valid data.

**Project scope:** One arm, robot only, no cameras, no dexterous hand, no gripper, no ROS, no trajectory execution, no robot-learning model, no demonstrations.

## 1. Hard safety restrictions (mandatory)

- This project is strictly READ-ONLY. Never issue robot enable/disable, motion, homing, free-drive, control-mode switching, Cartesian/joint commands, trajectories, force commands, or tool commands through software.
- Never bypass robot safety interlocks, permissions, controller licensing, firewall/security controls, or emergency stop procedures.
- Never operate robot hardware unattended or represent a software-only check as a hardware check.
- Steps involving robot power, network cabling, physical switches, teach pendant/Elements UI, RDK licensing, Auto (Remote) mode, or changes to controller settings **must be performed by an authorized person** following the robot manual and lab safety rules. Codex may explain and document these steps; it must not automate them.
- Do not run examples other than the explicitly permitted **state-display** script against real hardware, unless the user specifically approves after inspecting the code. Inspect the official script first and verify it contains no robot actuation before executing it.
- Make network/OS changes only after showing the proposed change and receiving approval. Never reconfigure an interface carrying the user's remote session. Do not turn off the host firewall globally. Do not run privileged commands automatically.
- If any prerequisite or real-world setting is unknown, pause that physical action and report the missing fact. Continue with independently safe software-only tasks.
- A successful ping is not proof that RDK Server is running or that Remote mode is active.

## 2. Read official documentation first

Consult and follow the CURRENT official documentation at execution time. Do not assume an outdated API, serial-number convention, Python version, or example filename.

Official sources:

1. RDK repository: https://github.com/flexivrobotics/flexiv_rdk
2. Repository README: https://github.com/flexivrobotics/flexiv_rdk/blob/main/README.md
3. Repository AI guidance (if present): https://github.com/flexivrobotics/flexiv_rdk/blob/main/llms.txt
4. Manual: https://www.flexiv.com/software/rdk/manual/index.html
5. First-time setup: https://flexiv.us/software/rdk/manual/v2.x/index.html
6. Set up robot: https://flexiv.us/software/rdk/manual/v2.x/set_up_the_robot.html
7. Activate RDK Server: https://flexiv.us/software/rdk/manual/v2.x/activate_rdk_server.html
8. Enter/exit Remote mode: https://flexiv.us/software/rdk/manual/v2.x/enter_and_exit_remote_mode.html
9. Connect user computer: https://flexiv.us/software/rdk/manual/v2.x/connect_user_computer.html
10. Python examples: https://github.com/flexivrobotics/flexiv_rdk/tree/main/example_py

Check whether these URLs still resolve. If a versioned manual path has changed, find its equivalent from the manual index. Read the robot's own supplied user manual and local laboratory instructions before advising physical operations. Record which RDK Git commit/tag, installed package version, and documentation revision you used.

Known reference facts, to VERIFY in current docs:

- Official README advertises Ubuntu 22.04+ and Windows 10+ with supported Python versions dependent on platform/release. Prefer **Python 3.10 64-bit** when compatible, rather than assuming the OS default Python is supported.
- Official Python package installation: `python -m pip install numpy spdlog flexivrdk` using a supported interpreter.
- Official example invocation: `python ./basics1_display_robot_states.py <ACTUAL_ROBOT_SERIAL_NUMBER>` from `flexiv_rdk/example_py`.
- The sample serial `Enlight-L-123456` is only a placeholder; **NEVER** use it as a real serial.
- Official default controller LAN behavior: User Port 1 static `192.168.2.100/24`; User Port 2 DHCP; General Port DHCP server at `192.168.100.1`. These are defaults, NOT guaranteed facts about this lab.
- For direct Ethernet, match robot MTU (documented default 1500).
- Standard RDK license covers reading states; the operator must verify licensing in Flexiv Elements.

## 3. Deliverables (create these files)

Create a new project directory named `flexiv_enlight_readonly_setup/` in the user's selected workspace (do NOT modify other existing robotics projects). Target structure:

```text
flexiv_enlight_readonly_setup/
├── README.md
├── .gitignore
├── requirements.txt
├── .env.example
├── config/
│   └── robot.example.env
├── docs/
│   ├── MANUAL_HARDWARE_CHECKLIST.md
│   ├── NETWORK_SETUP.md
│   ├── TROUBLESHOOTING.md
│   └── SETUP_REPORT.md
├── scripts/
│   ├── check_environment.py
│   ├── network_diagnostics.py
│   ├── check_rdk_import.py
│   └── run_official_state_example.py
├── tests/
│   └── test_offline_checks.py
├── third_party/
│   └── README.md
└── logs/
    └── .gitkeep
```

Clone official `flexiv_rdk` to `third_party/flexiv_rdk` **only if online access is available**. Otherwise keep a documented clone command and report the blocker. Do not modify the upstream repository or copy its code into a new file unless necessary; keep the read-only state demo as the official upstream script. Put temporary generated files and Python virtualenv in ignored locations.

Use UTF-8 Markdown, straightforward English, clearly marked commands for Windows PowerShell and Ubuntu Bash. Do not use LibreOffice.

## 4. Execution phases

### Phase A — Inspect environment (always first)

1. Print working directory, current OS, OS version, architecture, detected Python paths and versions, Git availability, and `pip` version.
2. Determine whether execution occurs on Windows, native Ubuntu, or Ubuntu inside VirtualBox/VMware/Hyper-V/another VM. Use passive detection (`uname`, `/etc/os-release`, `systemd-detect-virt`, platform information) rather than guessing. If uncertain, record `unknown`.
3. Determine whether the current environment has a real wired NIC, a USB-Ethernet passthrough, or a virtual bridged NIC; list interfaces, current IPs, routes, and MTUs without editing settings. Distinguish host interfaces from guest interfaces.
4. If developing inside a VM, explain that **NAT often prevents direct robot discovery / local-network communication**. Recommend bridging to the physical wired Ethernet NIC, or passing a USB Ethernet adapter directly to the guest, after checking the hypervisor and lab policy. Do not assume all bridge implementations forward discovery correctly. Test connectivity from the **Ubuntu guest** where the Python script will run; Windows host ping success is insufficient.
5. Create `docs/SETUP_REPORT.md` with environment, command outputs summarized, outstanding assumptions, and what remains unverified. Do not save sensitive identifiers, passwords, or unnecessary network details in public Git history.

### Phase B — Establish software-only project baseline

1. Create requested folders and files.
2. Use Python 3.10 64-bit where available and supported by latest upstream release; otherwise select another **explicitly supported** version, or document installation instructions and stop before dependencies.
3. Create a project-local `.venv` using the chosen interpreter:

Ubuntu example:
```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

Windows PowerShell example:
```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
```

4. Add direct dependencies in `requirements.txt`: `numpy`, `spdlog`, `flexivrdk`. Do not guess arbitrary pinned versions; if reproducible pins are needed, record the successfully installed exact versions in the setup report or a generated lock file. Install using `python -m pip install -r requirements.txt` only if internet/permission is available.
5. Verify module import in a subprocess with timeout and clear error handling; e.g. `python -c "import flexivrdk; print('RDK import OK')"`. **Importing** the module must not attempt a robot connection or movement.
6. Clone the official repository:
```bash
git clone https://github.com/flexivrobotics/flexiv_rdk.git third_party/flexiv_rdk
```
Check the repository README, `llms.txt` if available, and exact location/content of `example_py/basics1_display_robot_states.py`. Record the commit hash. If the file was renamed, locate the current equivalent and update the documentation; do not fabricate it.
7. Open and inspect the official state example. Verify there are NO calls commanding actuation. If there are side effects beyond reading, do NOT run it; report the issue and propose a strictly read-only alternative for user review.

### Phase C — Write manual robot-side checklist (no physical automation)

Create `docs/MANUAL_HARDWARE_CHECKLIST.md` with individual unchecked boxes and places for user confirmation:

- [ ] Robot model and controller identified; serial number recorded locally in a non-committed file.
- [ ] Hardware securely mounted, workspace cleared, safety conditions checked by authorized operator.
- [ ] Robot/controller power and Elements tablet set up according to Flexiv quick-start and lab manual.
- [ ] Flexiv Elements displays robot status and relevant sensor data.
- [ ] RDK Standard or Professional license verified in Elements.
- [ ] In Elements: Settings -> Remote Mode -> Ethernet; save, then perform prescribed power cycle (manual human action).
- [ ] Enter Auto (Remote) per the official **Enter and Exit Remote Mode** instructions, under authorized supervision. Emphasize that enabling Remote mode is distinct from merely configuring it. Do not automate the switch.
- [ ] Controller LAN port chosen and actual robot IP verified against lab configuration.
- [ ] Ethernet cable plugged into correct controller port, with no conflicts in the lab network.
- [ ] Robot actual serial number stored only in ignored local config and verified from controller/Elements/label.
- [ ] Authorized operator confirms it is safe to connect and run a state-display-only diagnostic.

State clearly: for this milestone, no RDK commands that enable, move, or switch controller modes are authorized. Some official hardware initialisation instructions may reference servo enable from Elements; these must be left to an authorized operator and should not be executed just to get state display working without checking necessity.

### Phase D — Network diagnostics (start passive; edits only with permission)

Create `docs/NETWORK_SETUP.md` explaining three connection options:

**Option 1 (recommended for simple direct connection):** PC Ethernet -> robot User Port 1. Factory default robot IP `192.168.2.100/24`; suggested unused PC IP `192.168.2.10/24` and no gateway. Treat both as **examples** and verify before use.

**Option 2:** PC and robot User Port 2 connected through same DHCP router; discover the assigned robot IP via approved controller/router procedures.

**Option 3:** Direct PC -> robot General Port; controller DHCP server defaults to `192.168.100.1`, PC uses DHCP.

For native Ubuntu, provide PASSIVE commands first:
```bash
ip -br addr
ip route
ip link show
ping -c 4 192.168.2.100
```

For Windows PowerShell:
```powershell
Get-NetAdapter
Get-NetIPAddress -AddressFamily IPv4
Get-NetIPInterface -AddressFamily IPv4
ping 192.168.2.100
```

For Ubuntu VM, provide host and guest checks separately. Explain bridged networking or USB network-adapter passthrough. Never claim a guest has a robot route merely because host ping works. Check route selection and whether packets use the correct interface. Note that some VM/USB adapters require host-side driver or bridge changes, to be done manually.

Build `scripts/network_diagnostics.py` as a safe cross-platform diagnostics tool:
- Accept `--robot-ip` (default should be blank or explicitly marked example; no hidden assumptions), `--timeout`, and optional `--output`.
- Validate the IP address using Python `ipaddress`.
- Print system, interfaces or the appropriate platform commands, route hints, and bounded `ping` result (4 pings maximum by default). Use `subprocess.run` with argv arrays, not `shell=True`, and with timeout.
- Diagnostics only: no NIC reconfiguration, firewall modification, network scanning, or robot API control.
- Distinguish DNS/invalid address, no route, no ping reply, and ping success. Do not claim ping success equals an operational RDK connection.
- If network interfaces are not discoverable, report instructions instead of failing mysteriously.

Document MTU requirement for direct connection (1500 if the controller is at the documented default), how to **inspect** it (`ip link show <interface>` or `netsh interface ipv4 show subinterfaces`) and how to ask the operator to change it safely if needed. Mention firewall allowlisting on lab/private networks rather than blanket disabling.

### Phase E — Read-only RDK test wrappers

Implement these scripts:

**`scripts/check_environment.py`**
- Read-only Python/OS/architecture/virtualization facts; detect supported interpreter versions using current official README; never modify the system.
- Give a clear PASS / WARN / FAIL-style report, but avoid declaring unsupported until the docs/version are verified.

**`scripts/check_rdk_import.py`**
- Import `flexivrdk`, display package version with `importlib.metadata.version('flexivrdk')` if possible, and print import success.
- No Robot object creation and no hardware interaction.

**`scripts/run_official_state_example.py`**
- This is a cautious wrapper around the **unchanged** upstream `third_party/flexiv_rdk/example_py/basics1_display_robot_states.py`.
- Require actual serial number through `--serial` or environment variable `FLEXIV_ROBOT_SERIAL`; never default to `Enlight-L-123456`.
- Require explicit opt-in flag `--connect-readonly` to prevent accidental real-hardware access.
- Before running, check that the official script exists and has been manually inspected for read-only behavior. Maintain an allowlist of that exact script path. Prefer recording upstream Git commit and script hash in report. If source has changed after verification, refuse and require re-inspection.
- Use the same venv Python (`sys.executable`) for the script; never execute via privileged access.
- Print the exact invocation and intended behavior, then run with subprocess and allow `Ctrl+C` clean stop. Support `--timeout-seconds` to stop after a short demonstration period, and capture readable errors.
- Ensure stdout/stderr are visible. Make log recording optional and exclude device identifiers where possible. Never automatically upload data.
- No robot motion, mode change, or API commands in the wrapper.

**`tests/test_offline_checks.py`**
- Offline tests for serial validation, IPv4 validation, command-building, failure diagnostics, timeout, and missing official example.
- Mock subprocess and filesystem as necessary. Tests must NOT require an actual Flexiv robot or connect to a robot.

**Configuration**
- `.env.example` and `config/robot.example.env` include placeholder keys only, e.g. `FLEXIV_ROBOT_SERIAL=REPLACE_WITH_ACTUAL_SERIAL`, `FLEXIV_ROBOT_IP=REPLACE_WITH_VERIFIED_IP`. Avoid committing actual serials, IPs beyond illustrative default docs, or other lab-specific data.
- `.gitignore` includes `.venv/`, `__pycache__/`, `.pytest_cache/`, `logs/*` but preserves `logs/.gitkeep`, `.env`, `config/robot.env`, local reports with sensitive host information if appropriate.

### Phase F — Verification and issue triage

Perform these in this order, reporting status for each:

1. OFFLINE: Validate directory layout and README instructions.
2. OFFLINE: Python syntax/compilation checks.
3. OFFLINE: Run test suite. Use `unittest` from the standard library unless there is a strong reason for another test dependency.
4. OFFLINE: Attempt import test if dependencies installed. If pip/network unavailable, document exact reason.
5. OFFLINE: Confirm official script exists and inspect for read-only calls.
6. NETWORK: Only when the operator supplies a verified robot IP, run bounded ping from **execution environment** (guest, if running in VM).
7. HARDWARE: Only after the operator confirms licensing, Remote Mode, physical safety, IP and actual serial, show the exact official command and request approval to execute the read-only state-display example. Run only if environment permits real hardware contact and permission is explicit.
8. Observe actual live state lines and check for plausible nonempty values; do NOT invent joint values. State what was verified and what was not.

Suggested official test command from repository root after activating venv:

Ubuntu:
```bash
cd third_party/flexiv_rdk/example_py
python basics1_display_robot_states.py "<ACTUAL_ROBOT_SERIAL>"
```

Windows PowerShell:
```powershell
cd third_party\flexiv_rdk\example_py
python .\basics1_display_robot_states.py "<ACTUAL_ROBOT_SERIAL>"
```

Alternatively run the approved guarded wrapper:
```bash
python scripts/run_official_state_example.py --serial '<ACTUAL_ROBOT_SERIAL>' --connect-readonly --timeout-seconds 15
```

An approved operator should know the E-stop location; the approved demo must not move the robot. If the script does not run or Python import fails, do not troubleshoot by trying a motion example.

### Phase G — Troubleshooting matrix

Create `docs/TROUBLESHOOTING.md` with symptom -> test -> likely cause -> safe corrective action, including:

- `python: command not found` / unsupported Python ABI / package wheel unavailable.
- `No module named flexivrdk` due to wrong interpreter/venv.
- Missing Git and failed source clone.
- Missing upstream example or changed source/API.
- Guest in NAT mode; host has connectivity but guest cannot reach robot.
- Wrong interface or subnet; duplicate static IP; wrong controller LAN port.
- Controller offline; `ping` timeout; dropped packets/MTU mismatch.
- ICMP might be filtered; no ping response does not by itself prove hardware is absent.
- Windows/Ubuntu firewall blocking RDK traffic; advise narrowly scoped approved rules.
- Wrong robot serial; no discovery or session connection.
- RDK license unavailable; Remote Mode configured but not entered; robot not ready.
- If connection succeeds but no valid states: inspect messages/logs and official compatibility/release notes before any higher-risk action.

Do not recommend blanket firewall disable, installing random unverified packages, or running commands with `sudo` unless strictly needed and authorized.

## 5. Required README instructions

`README.md` must describe: goal, supported environments (as actually observed), safety limits, exact step-by-step software setup, manual hardware checklist link, three network modes, Ubuntu VM bridge/passthrough advice, Python RDK import test, the upstream script invocation, wrapper invocation, stop with Ctrl+C, and troubleshooting links.

Keep a short **'Fast path'** section for users with confirmed Ubuntu 22.04+ and Python 3.10, and a separate Windows PowerShell section. Do not mix Bash and PowerShell commands in a single copy/paste block.

## 6. Codex reporting protocol

After each phase, update `docs/SETUP_REPORT.md` using a table:

| Item | Result | Evidence | Required user action |
|---|---|---|---|
| OS / VM detection | PASS / WARN / BLOCKED | ... | ... |
| Python venv | ... | ... | ... |
| RDK import | ... | ... | ... |
| Official repo example verified | ... | ... | ... |
| Network connectivity (from execution env) | ... | ... | ... |
| License and Auto (Remote) | UNVERIFIED until operator confirms | ... | ... |
| Live robot state display | UNVERIFIED until physically observed | ... | ... |

End your response with:
1. Paths of created files.
2. Exact commands that worked and exact tests executed.
3. Any errors, with evidence.
4. Manual-only steps the user must complete.
5. **The next single safe command** to execute.

**Do not say 'setup complete' until the live state-display verification really succeeded.** If hardware is inaccessible, finish all safe offline work and make the result immediately useful.

## 7. Acceptance criteria

- [ ] All requested project files exist and contain usable, specific instructions.
- [ ] No software created here contains robot movement or enable/control-mode instructions.
- [ ] Python environment is reproducible and RDK import checked when possible.
- [ ] Upstream repo and official state example examined, not replaced with fabricated code.
- [ ] VM networking addressed explicitly; host and guest checks separated.
- [ ] Network commands are non-destructive by default.
- [ ] Serial number cannot silently fall back to example value.
- [ ] Only an operator-approved read-only hardware example can run.
- [ ] Offline tests pass (if dependencies present).
- [ ] Report honestly distinguishes observed results from unverified steps.

## 8. Start now

Begin with passive environment inspection and current official-doc/repository review. Then create the files, implement tests/scripts, run offline verification, and write a concise report. Do not ask broad questions that block safe preparation. Ask the operator only for a specific necessary fact when reaching the actual hardware connection step (e.g. verified real serial number, robot IP, and confirmation of RDK license/Auto Remote mode). Do not run any motion-related example.
