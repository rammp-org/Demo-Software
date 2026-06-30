#!/usr/bin/env python3
"""Probe all input devices and print key-down events with their source node + keycode.

Use this to discover which /dev/input/eventX node a keypad emits on and what
keycodes its keys send (e.g. for configuring keyboard_driver's device_path /
device_name and KEY_TO_ACTION map).

Run:  python3 scripts/keypad_probe.py
Then press the keys you care about. Ctrl-C to stop.

Needs python3-evdev and read access to /dev/input/event* (user in 'input' group,
or run with sudo).
"""

from select import select

import evdev
from evdev import ecodes


def main():
    paths = evdev.list_devices()
    if not paths:
        print(
            "No input devices found. Try sudo, or add your user to the 'input' group."
        )
        return

    devs = {}
    print("Listening on:")
    for path in paths:
        try:
            dev = evdev.InputDevice(path)
        except OSError as e:
            print(f"  (skip {path}: {e})")
            continue
        devs[dev.fd] = dev
        print(f"  {dev.path} -> {dev.name}")

    print("\nNow press your keys (key-down events only). Ctrl-C to stop.\n")
    try:
        while True:
            r, _, _ = select(devs, [], [])
            for fd in r:
                for event in devs[fd].read():
                    if event.type == ecodes.EV_KEY and event.value == 1:
                        dev = devs[fd]
                        print(
                            f"{dev.path}  {dev.name!r}  "
                            f"code={event.code}  name={ecodes.KEY.get(event.code)}"
                        )
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
