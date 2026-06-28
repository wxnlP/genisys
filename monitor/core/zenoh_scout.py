#!/usr/bin/env python3
"""Quick zenoh scout + listen test."""
from __future__ import annotations

import time
import zenoh


def main() -> None:
    print("Opening zenoh session (peer mode)...")
    config = zenoh.Config()
    config.insert_json5("mode", "'peer'")
    config.insert_json5("scouting/multicast/enabled", "true")

    session = zenoh.open(config)
    zid = session.info().zid()
    print(f"Session opened, zid={zid}")

    # Listen on a broad pattern
    count = [0]

    def on_sample(s):
        count[0] += 1
        raw = s.payload.to_bytes()
        ke = s.key_expr()
        print(f"[{count[0]}] key_expr={ke}  payload={len(raw)}B  "
              f"hex={raw[:24].hex()}...")

    # Try various key expressions
    for ke in ["geniarm/joint_state",
               "geniarm/**",
               "**"]:
        try:
            session.declare_subscriber(ke, on_sample)
            print(f"Subscribed: {ke}")
        except Exception as e:
            print(f"FAIL [{ke}]: {e}")

    print("\nListening (Ctrl+C to stop)...\n")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()
        print(f"\nTotal samples: {count[0]}")


if __name__ == "__main__":
    main()
