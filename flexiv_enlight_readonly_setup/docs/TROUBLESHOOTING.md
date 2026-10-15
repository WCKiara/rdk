# Troubleshooting

| Symptom | Test | Likely cause | Safe corrective action |
|---|---|---|---|
| Python missing or wheel unavailable | `python --version`; environment check; pip error | Missing interpreter or unsupported ABI/platform | Use a current upstream-supported 64-bit Python; do not install random packages |
| No module named flexivrdk | `python scripts/check_rdk_import.py` | Wrong interpreter/venv or failed installation | Use project `.venv` Python; install requirements locally |
| Missing Git / clone failure | `git --version`; clone output | Git missing, offline network, proxy restriction | Document blocker; install Git with appropriate approval or retry official clone |
| Missing/changed official example | Review wrapper error and source review | Wrong revision, changed source or API | Inspect upstream exact source and compatibility; never bypass hash guard |
| Official wrapper refuses current source | `docs/source_review.json` | Upstream calls fault clearing and servo enable | Keep refusal; review proposed strictly read-only alternative and obtain separate approval |
| Host ping works, Ubuntu guest fails | Compare host/guest route and NIC | NAT, bridge filtering, missing USB passthrough | Operator reviews bridge/passthrough setup; test again from guest |
| Wrong interface/subnet | Route to verified IP; actual controller settings | Old static profile or wrong controller port | Verify port label and actual subnet; propose exact NIC change for approval |
| Duplicate static IP | Operator checks controller/router assignments | Another device has PC address | Choose confirmed-unused address with approved change |
| No ping reply | Bounded diagnostic, route and link lights | Controller offline, wrong IP/port, ICMP filtering | Operator checks power/label/IP; lack of ICMP alone does not prove absence |
| Packet loss / large packets fail | Link/MTU inspection | MTU mismatch or cabling | Operator checks cable and controller MTU; approve any PC change |
| RDK traffic blocked | Inspect scoped firewall policy | Host/guest or lab firewall | Request narrow allowlist; never disable globally |
| Discovery/session fails | Verified serial and official errors | Wrong serial, subnet/discovery filtering, no license, wrong software version | Read identifiers from Elements/label and current compatibility/release notes |
| Remote configuration exists but no session | Operator checks status in Elements | Auto (Remote) not entered | Authorized operator follows supplied manual; software does not switch mode |
| No valid state samples | Review nonempty finite q and changing timestamps | Incompatible software, missing state data, wrong group | Read compatibility/errors; never try a motion/enable example to fix it |
| Sandbox blocks `ip`, ping or packet capture | Diagnostic stderr | Restricted netlink/raw sockets or account permissions | Run passive scripts in normal terminal; do not automate privileged capture |

The supplied robot user manual and lab instructions take precedence for physical work. No fault clearing, servo commands, tool commands, or mode changes are part of troubleshooting here.

Sources: [current upstream README](https://github.com/flexivrobotics/flexiv_rdk/blob/main/README.md), [manual index](https://flexiv.us/software/rdk/manual/v2.x/index.html), [software compatibility](https://flexiv.us/software/rdk/manual/v2.x/robot_software_compatibility.html), [release notes](https://flexiv.us/software/rdk/manual/v2.x/release_notes.html).
