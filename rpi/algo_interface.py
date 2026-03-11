"""
Algorithm Service Interface Module

Communicates with the pathfinding REST API running on the PC.

Endpoint:
    POST http://<PC_IP>:<port>/path

Request body:
    {
        "obstacles": [
            {"x": 9, "y": 9, "d": 0, "obstacleNumber": 1}
        ],
        "retrying": false,
        "robot_x": 1,
        "robot_y": 1,
        "robot_dir": 0
    }

Response:
    {
        "data": {
            "commands": ["FR00", "FW10", "SNAP1_C", ..., "FN"],
            "distance": 46.0,
            "path": [...]
        },
        "error": null
    }

Usage:
    algo = AlgoInterface(host="<PC_IP>")

    obstacles = [
        {"x": 12, "y": 12, "d": 0, "obstacleNumber": 1},
    ]

    result = algo.request_path(obstacles, robot_dir=0)
    if result:
        for cmd in result["data"]["commands"]:
            print(cmd)

Context manager is supported:
    with AlgoInterface(host="<PC_IP>") as algo:
        ...

Standalone test:
    python algo_interface.py --host <PC_IP> --port 15001
"""

import argparse
import json
import sys
from typing import Any, Optional

import requests


class AlgoInterface:
    """
    HTTP client for the pathfinding algorithm service on the PC.

    Sends obstacle data to the algo service and receives back a flat list
    of movement commands for the robot to follow.
    """

    DEFAULT_PORT = 15001
    DEFAULT_TIMEOUT = 30.0

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout = timeout
        self._base_url = f"http://{self._host}:{self._port}"
        self._session: Optional[requests.Session] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        if self._session is not None:
            print("[ALGO] Session already open")
            return True

        self._session = requests.Session()
        self._session.headers.update({"Content-Type": "application/json"})
        print(f"[ALGO] Session ready -> {self._base_url}")
        return True

    def stop(self) -> None:
        if self._session:
            self._session.close()
            self._session = None
            print("[ALGO] Session closed")

    @property
    def is_running(self) -> bool:
        return self._session is not None

    # ------------------------------------------------------------------
    # Pathfinding request
    # ------------------------------------------------------------------

    def request_path(
        self,
        obstacles: list[dict[str, Any]],
        robot_x: int = 1,
        robot_y: int = 1,
        robot_dir: int = 0,
        retrying: bool = False,
    ) -> Optional[dict[str, Any]]:
        """
        Request a pathfinding solution.

        Args:
            obstacles: List of obstacle dicts, each with
                       ``x``, ``y``, ``d``, ``obstacleNumber``.
            robot_x:   Robot starting x (default 1).
            robot_y:   Robot starting y (default 1).
            robot_dir: Robot starting direction (0=N, 2=E, 4=S, 6=W).
            retrying:  Whether this is a retry attempt.

        Returns:
            Parsed JSON response on success, or None on failure.
            Access commands via ``result["data"]["commands"]``.
        """
        if not self._session:
            print("[ALGO] Not started – call start() first")
            return None

        payload = {
            "obstacles": obstacles,
            "retrying": retrying,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_dir": robot_dir,
        }

        url = f"{self._base_url}/path"
        print(f"[ALGO] POST {url} ({len(obstacles)} obstacle(s))")

        try:
            resp = self._session.post(url, json=payload, timeout=self._timeout)
        except requests.ConnectionError:
            print(f"[ALGO] Connection refused – is the service running at {self._base_url}?")
            return None
        except requests.Timeout:
            print(f"[ALGO] Request timed out after {self._timeout}s")
            return None
        except requests.RequestException as e:
            print(f"[ALGO] Request error: {e}")
            return None

        if resp.status_code != 200:
            print(f"[ALGO] HTTP {resp.status_code}: {resp.text[:200]}")
            return None

        try:
            data = resp.json()
        except ValueError:
            print("[ALGO] Invalid JSON in response")
            return None

        if data.get("error"):
            print(f"[ALGO] Service error: {data['error']}")
            return None

        commands = data.get("data", {}).get("commands", [])
        distance = data.get("data", {}).get("distance", 0)
        print(f"[ALGO] Received {len(commands)} command(s), distance={distance}")

        return data

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "AlgoInterface":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.stop()
        return False


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Algo Interface (standalone test)")
    parser.add_argument("--host", required=True, help="PC IP address running the algo service")
    parser.add_argument("--port", type=int, default=AlgoInterface.DEFAULT_PORT, help="Algo service port")
    args = parser.parse_args()

    sample_obstacles = [
        {"x": 9, "y": 9, "d": 0, "obstacleNumber": 1},
    ]

    with AlgoInterface(host=args.host, port=args.port) as algo:
        print(f"Requesting path for {len(sample_obstacles)} obstacle(s)...")
        result = algo.request_path(sample_obstacles)
        if result:
            print("\nFull response:")
            print(json.dumps(result, indent=2))
        else:
            print("\nNo result received.")
