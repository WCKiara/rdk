# Network setup

Inspect first. Do not edit a profile, MTU, route, firewall, or host/guest bridge until the exact proposed change is shown and approved. Do not reconfigure the remote-session interface or disable a firewall globally. No scan or network edits are built into the diagnostics tool.

## Connection options

| Connection | Controller default (illustrative) | PC configuration |
|---|---|---|
| PC Ethernet → User Port 1 | Static `192.168.2.100/24` | Example unused `192.168.2.10/24`, no gateway; verify actual subnet first |
| PC and User Port 2 → same router | DHCP address | DHCP; read robot IP from approved Elements/router procedure |
| PC Ethernet → General Port | DHCP server `192.168.100.1` | DHCP; verify controller's actual setting |

These are documented factory defaults, not this laboratory's verified configuration. Port identity must be checked from its physical label.

## Passive Ubuntu checks

```bash
ip -br addr
ip route
ip link show
python scripts/network_diagnostics.py
```

After actual controller IP is confirmed:

```bash
ip route get '<VERIFIED_ROBOT_IP>'
python scripts/network_diagnostics.py --robot-ip '<VERIFIED_ROBOT_IP>' --timeout 2
```

Illustrative factory-default check, only after verifying that address:

```bash
ping -c 4 192.168.2.100
```

Confirm route uses the intended Ethernet interface. Stop if it selects Wi-Fi or an unrelated network; request a reviewed network change. Four probes maximum are issued by the tool. Invalid literals/DNS names, missing commands, permission restrictions, unreachable routes/peers, timeouts, no replies, and successful ping are reported separately. ICMP may be filtered; ping success does not verify the RDK server, licensing, Remote mode, or serial.

## Passive Windows PowerShell checks

```powershell
Get-NetAdapter
Get-NetIPAddress -AddressFamily IPv4
Get-NetIPInterface -AddressFamily IPv4
python scripts\network_diagnostics.py
```

After verifying the controller address, a bounded illustrative default check is:

```powershell
ping -n 4 192.168.2.100
```

## Host and guest checks

On a Windows host run the PowerShell checks above. Inside an Ubuntu guest run all Ubuntu checks and the bounded ping using the guest's own Python environment. NAT often prevents direct local discovery. Bridging to physical Ethernet or USB Ethernet passthrough may be suitable after checking hypervisor drivers and lab policy. Some bridges still filter discovery; test guest connectivity explicitly. Host ping alone is insufficient.

## MTU and firewall

Direct Ethernet requires matching controller/PC MTU; the documented factory default is 1500. The observed PC `eno2` MTU is 1500; controller MTU is unverified.

```bash
ip link show eno2
```

```powershell
netsh interface ipv4 show subinterfaces
```

If needed, ask the operator/network administrator to review the precise MTU change. For firewall issues use narrowly scoped program/network allowlisting on the approved lab/private network, based on current official traffic requirements. Do not disable the host firewall.

## Observed mismatch and pending proposal

The existing Ethernet profile was activated at the user's request before this guide was supplied. Local profile/IP details are recorded only in the ignored setup report. Its subnet differs from the factory-default User Port 1 subnet. A hypothetical unused PC `192.168.2.10/24` is appropriate only if the operator verifies that the controller uses `192.168.2.100/24`, confirms no address conflict, and approves a separate Ethernet-only profile with no gateway and MTU 1500. No such change has been made. Preserve the existing profile for rollback and leave the remote-session interface untouched.

Source: [Flexiv Connect User Computer](https://flexiv.us/software/rdk/manual/v2.x/connect_user_computer.html), reviewed 2026-10-12.
