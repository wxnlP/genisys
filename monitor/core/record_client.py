"""Minimal client for the AIMRT record_playback plugin RPC service."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any
from urllib import error, request


class RecordPlaybackClient:
    """Call the record_playback plugin over HTTP JSON-RPC style endpoints."""

    def __init__(self, base_url: str | None = None, output_dir: str | None = None) -> None:
        self._base_url = (base_url or os.environ.get("AIMRT_RECORD_RPC_URL") or "http://127.0.0.1:50080").rstrip("/")
        self._output_dir = Path(output_dir or os.environ.get("AIMRT_RECORD_OUTPUT_DIR") or "./recordings").resolve()
        self._output_dir.mkdir(parents=True, exist_ok=True)

    @property
    def base_url(self) -> str:
        return self._base_url

    def set_base_url(self, base_url: str) -> None:
        self._base_url = base_url.rstrip("/")

    @property
    def default_output_dir(self) -> str:
        return str(self._output_dir)

    def _build_url(self, method_name: str) -> str:
        service_path = "aimrt.protocols.record_playback_plugin.RecordPlaybackService"
        return f"{self._base_url}/rpc/{service_path}/{method_name}"

    def _post_json(self, method_name: str, payload: dict[str, Any]) -> dict[str, Any]:
        data = json.dumps(payload).encode("utf-8")
        req = request.Request(self._build_url(method_name), data=data, headers={"content-type": "application/json"}, method="POST")
        try:
            with request.urlopen(req, timeout=5) as resp:
                body = resp.read().decode("utf-8")
        except error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="ignore")
            raise RuntimeError(f"record_playback plugin returned HTTP {exc.code}: {detail}") from exc
        except error.URLError as exc:
            raise RuntimeError(f"cannot reach record_playback plugin at {self._base_url}: {exc.reason}") from exc

        if not body:
            return {}
        try:
            return json.loads(body)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"invalid JSON response from record_playback plugin: {body}") from exc

    def start_record(self, action_name: str, preparation_duration_s: int = 0, record_duration_s: int = 0) -> dict[str, Any]:
        payload = {
            "action_name": action_name,
            "preparation_duration_s": preparation_duration_s,
            "record_duration_s": record_duration_s,
        }
        rsp = self._post_json("StartRecord", payload)
        if rsp.get("common_rsp", {}).get("code") not in (None, 0, "0"):
            raise RuntimeError(rsp.get("common_rsp", {}).get("msg") or "StartRecord failed")
        return rsp

    def stop_record(self, action_name: str) -> dict[str, Any]:
        payload = {"action_name": action_name}
        last_error: RuntimeError | None = None

        for attempt in range(2):
            try:
                rsp = self._post_json("StopRecord", payload)
            except RuntimeError as exc:
                last_error = exc
                if attempt == 0 and "has not started yet" in str(exc).lower():
                    time.sleep(0.2)
                    continue
                raise

            if rsp.get("code") not in (None, 0, "0"):
                msg = rsp.get("msg") or "StopRecord failed"
                if attempt == 0 and any(token in msg.lower() for token in ("has not started yet", "stop operation is not allowed")):
                    time.sleep(0.2)
                    continue
                raise RuntimeError(msg)

            return rsp

        if last_error is not None:
            raise last_error
        raise RuntimeError("StopRecord failed")

    def start_playback(self, action_name: str, skip_duration_s: int = 0, play_duration_s: int = 0) -> dict[str, Any]:
        payload = {
            "action_name": action_name,
            "skip_duration_s": skip_duration_s,
            "play_duration_s": play_duration_s,
        }
        rsp = self._post_json("StartPlayback", payload)
        if rsp.get("code") not in (None, 0, "0"):
            raise RuntimeError(rsp.get("msg") or "StartPlayback failed")
        return rsp

    def stop_playback(self, action_name: str) -> dict[str, Any]:
        payload = {"action_name": action_name}
        rsp = self._post_json("StopPlayback", payload)
        if rsp.get("code") not in (None, 0, "0"):
            raise RuntimeError(rsp.get("msg") or "StopPlayback failed")
        return rsp
