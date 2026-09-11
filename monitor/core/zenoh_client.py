"""
Zenoh subscriber client for AIMRT channels.

Subscribes to a zenoh key-expression and decodes protobuf
JointStateArray / JointCommandArray messages based on the
configured topics in ``config.yaml``.
"""

from __future__ import annotations

import hashlib
import logging
import struct
from collections import deque
from typing import Dict, List, Optional
from urllib.parse import unquote

import zenoh
from PyQt5 import QtCore

from monitor.core.config import config
from monitor.core.msg_decode import decode_json as _decode_json_bytes
from monitor.core.msg_decode import decode_pb as _decode_pb_bytes
from monitor.core.msg_decode import quat_to_rpy as _quat_to_rpy  # noqa: F401

logger = logging.getLogger(__name__)

SUBSCRIBE_KEY = "**"

# Rolling window of recent payload fingerprints — >500ms apart → same seq OK
_DUP_WINDOW = 256

def _resolve_source(topic_key: str) -> str | None:
    """Return the source name (e.g. ``'joint_state'``) if *topic_key*
    matches one of the configured topic expressions, else ``None``.

    The key is URL-decoded before matching because zenoh ``**``
    subscriptions deliver keys with ``%2F``-encoded slashes.
    """
    decoded = unquote(topic_key)
    for source, expr in config.topics.items():
        if expr in decoded:
            return source
    return None


class ZenohSubscriber(QtCore.QObject):
    """Thin wrapper around a zenoh subscriber that emits Qt signals."""

    # Emitted whenever a full protobuf message is decoded.
    data_received = QtCore.pyqtSignal(dict)

    # Emitted when the subscriber receives *any* raw data (for debugging)
    raw_data_received = QtCore.pyqtSignal(str, int)  # key_expr, payload_len

    connection_changed = QtCore.pyqtSignal(bool)
    error_occurred = QtCore.pyqtSignal(str)

    def __init__(
        self,
        zenoh_config_path: Optional[str] = None,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._config_path = zenoh_config_path
        self._session: Optional[zenoh.Session] = None
        self._subscribers: List[zenoh.Subscriber] = []
        self._active = False
        self._msg_count = 0
        self._source_counts: dict[str, int] = {}   # per-source message counter
        self._source_logged: set[str] = set()       # logged first message per source
        # Rolling fingerprint window for deduplication across subscribers
        self._seen_fingerprints: deque[bytes] = deque(maxlen=_DUP_WINDOW)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> bool:
        if self._active:
            logger.warning("ZenohSubscriber: already running")
            return True

        try:
            if self._config_path:
                zenoh_cfg = zenoh.Config.from_file(self._config_path)
            else:
                zenoh_cfg = zenoh.Config()
                zenoh_cfg.insert_json5("mode", "'peer'")
                zenoh_cfg.insert_json5("scouting/multicast/enabled", "true")
                endpoints = config.aimrt_connect_endpoints
                if endpoints:
                    zenoh_cfg.insert_json5(
                        "connect/endpoints",
                        "[" + ", ".join(f'"{ep}"' for ep in endpoints) + "]",
                    )
                    logger.info("Zenoh connect endpoints: %s", endpoints)

            self._session = zenoh.open(zenoh_cfg)
        except Exception as exc:
            logger.error("Failed to open zenoh session: %s", exc)
            self.error_occurred.emit(f"Zenoh session open failed: {exc}")
            return False

        # Subscribe to the catch-all key expression so we get everything.
        try:
            sub = self._session.declare_subscriber(SUBSCRIBE_KEY, self._on_sample)
            self._subscribers.append(sub)
            logger.info("Subscribed: %s  (catch-all)", SUBSCRIBE_KEY)
        except Exception as exc:
            logger.error("Cannot subscribe to '%s': %s", SUBSCRIBE_KEY, exc)
            self.error_occurred.emit(f"Failed to subscribe: {exc}")
            self._session.close()
            self._session = None
            return False

        # Subscribe to each configured topic.  Zenoh may reject key
        # expressions with a leading slash, so we try both forms.
        subscribed_topics: list[str] = []
        for source, expr in config.topics.items():
            candidates = [expr]
            if expr.startswith("/"):
                candidates.append(expr[1:])
            else:
                candidates.append(f"/{expr}")
            ok = False
            for ke in candidates:
                try:
                    sub = self._session.declare_subscriber(ke, self._on_sample)
                    self._subscribers.append(sub)
                    subscribed_topics.append(f"{source} ({ke})")
                    ok = True
                    break
                except Exception:
                    pass
            if not ok:
                logger.warning("Subscription failed: %s — no matching key expr", expr)

        self._active = True
        self.connection_changed.emit(True)
        logger.info("ZenohSubscriber: listening  (%s)",
                     ", ".join(subscribed_topics))
        return True

    def stop(self) -> None:
        self._active = False
        for sub in self._subscribers:
            try:
                sub.undeclare()
            except Exception:
                pass
        self._subscribers.clear()
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None
        # Summary of messages received per source
        if self._source_counts:
            summary = "  ".join(f"{s}={c}" for s, c in self._source_counts.items())
            logger.warning("Zenoh Subscriber stopped  (total=%d  %s)", self._msg_count, summary)
        else:
            logger.warning("Zenoh Subscriber stopped  (total=%d)", self._msg_count)
        self.connection_changed.emit(False)

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def message_count(self) -> int:
        return self._msg_count

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Deduplication
    # ------------------------------------------------------------------

    def _is_duplicate(self, payload: bytes) -> bool:
        """Return ``True`` if *payload* was already seen recently.

        Multiple subscribers (``**`` + individual topic expressions) will
        receive the same message — we keep a rolling fingerprint window
        to skip duplicates.
        """
        fp = hashlib.md5(payload).digest()
        if fp in self._seen_fingerprints:
            return True
        self._seen_fingerprints.append(fp)
        return False

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------

    def _on_sample(self, sample: zenoh.Sample) -> None:
        """Zenoh callback – runs on the zenoh I/O thread."""
        try:
            payload: bytes = sample.payload.to_bytes()
            key: str = str(sample.key_expr)

            # Skip duplicates (same message arriving via different subscriptions)
            if self._is_duplicate(payload):
                return

            # Determine which configured topic this message matches
            source = _resolve_source(key)
            if source is None:
                # Log the first few unknown keys so the user can see if a
                # configured topic expression doesn't match the wire key.
                if self._msg_count < 3:
                    logger.info("Unknown source: %s", unquote(key))
                return

            self._msg_count += 1
            self._source_counts[source] = self._source_counts.get(source, 0) + 1

            # Log the first message from each source so we know data is flowing
            if source not in self._source_logged:
                self._source_logged.add(source)
                decoded_key = unquote(key)
                logger.info("Data flowing: %s  topic=%s", decoded_key, source)


            # Parse AIMRT channel framing header, then protobuf-decode the body.
            try:
                # --- AIMRT zenoh wire format (little-endian) ---
                # [4B] uint32  data_size  (payload size after this field)
                # [1B] uint8   ser_type_len
                # [N]  chars   ser_type    (e.g. "pb")
                # [1B] uint8   ctx_meta_count
                # [N×]: uint16_le key_len, key, uint16_le val_len, val
                # [remaining]   raw protobuf bytes
                # ---------------------------------------------------
                data_size = struct.unpack_from("<I", payload, 0)[0]
                if data_size + 4 > len(payload) or data_size < 2:
                    raise ValueError(f"Invalid data_size: {data_size}")
                pos = 4

                # serialization type
                ser_type_len = payload[pos]
                pos += 1
                ser_type = payload[pos:pos + ser_type_len].decode("ascii", errors="replace")
                pos += ser_type_len

                # context meta key-value pairs
                ctx_meta_count = payload[pos]
                pos += 1
                for _ in range(ctx_meta_count):
                    key_len = struct.unpack_from("<H", payload, pos)[0]
                    pos += 2
                    pos += key_len
                    val_len = struct.unpack_from("<H", payload, pos)[0]
                    pos += 2
                    pos += val_len

                pb_bytes = payload[pos:4 + data_size]

                if ser_type == "pb":
                    result = self._decode_pb(source, pb_bytes)
                    if result is not None:
                        result["source"] = source
                        self.data_received.emit(result)

                elif ser_type == "json":
                    result = self._decode_json(source, pb_bytes)
                    if result is not None:
                        result["source"] = source
                        self.data_received.emit(result)

                else:
                    if self._msg_count == 1:
                        logger.debug(
                            "Unknown serialization type '%s' for %s (%dB)",
                            ser_type, key, len(payload),
                        )

            except Exception:
                if self._msg_count <= 3:
                    logger.debug(
                        "AIMRT-framed decode failed for %s (%dB), "
                        "first 40 bytes: %s",
                        key, len(payload), payload[:40].hex(),
                    )
                    self.raw_data_received.emit(
                        f"{key} (decode-fail)", len(payload),
                    )

        except Exception as exc:
            logger.error("_on_sample error: %s", exc)

    # ------------------------------------------------------------------
    # Protobuf / JSON decoders — shared with the AIMRT transport backend
    # ------------------------------------------------------------------

    @staticmethod
    def _decode_pb(source: str, pb_bytes: bytes) -> dict | None:
        """Decode protobuf bytes according to *source* type."""
        return _decode_pb_bytes(source, pb_bytes)

    @staticmethod
    def _decode_json(source: str, json_bytes: bytes) -> dict | None:
        """Decode a JSON-encoded AIMRT payload according to *source* type."""
        return _decode_json_bytes(source, json_bytes)
