# Manual hardware checklist

Authorized operator: __________ Date/time: __________

Use the supplied Flexiv quick-start/user manual and lab procedures first. They were not available to the assistant. These steps are manual; the software does not change controller state. No robot enable, movement, fault clearing, or control-mode change is authorized for this milestone.

- [ ] Model/controller identified; actual serial recorded in ignored local configuration.
- [ ] Mounting secure, workspace clear, lab safety checks complete, E-stop location known.
- [ ] Controller/robot power and Elements tablet set up per supplied manuals.
- [ ] Elements displays robot status and sensor data.
- [ ] RDK Standard or Professional license verified in Elements.
- [ ] If configuration is necessary: Elements Settings → Remote Mode → Ethernet saved and prescribed power cycle completed manually under lab rules.
- [ ] Auto (Remote) entered by the operator following the official Remote-mode instructions. Configuring Ethernet Remote mode alone does not enter it.
- [ ] Controller LAN port label physically checked; actual IP/subnet verified from Elements/controller configuration.
- [ ] Cable connected to that port, link lights checked, no address conflicts.
- [ ] Actual robot serial verified from Elements/controller/label and stored only locally.
- [ ] Operator inspected the exact diagnostic source and approved state-display-only contact.
- [ ] Compatibility of installed RDK with actual robot software verified.

The official setup page includes servo-enable UI guidance. Do not perform that merely to make this diagnostic run; the authorized operator must determine necessity from the supplied manuals. The current official state example enables servos in software and therefore cannot be used here. Review `scripts/proposed_readonly_states.py` as an alternative; do not run it before separate approval.

Sources (reviewed 2026-10-12): [robot setup](https://flexiv.us/software/rdk/manual/v2.x/set_up_the_robot.html), [license/server](https://flexiv.us/software/rdk/manual/v2.x/activate_rdk_server.html), [Remote mode](https://flexiv.us/software/rdk/manual/v2.x/enter_and_exit_remote_mode.html).
