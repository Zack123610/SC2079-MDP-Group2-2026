"""
Algorithm Service Interface Module

Communicates with the pathfinding REST API running on the PC (default port 5001).

The algo service accepts a robot configuration (constant across runs) and a
list of obstacles (varies per run), then returns an ordered list of path
segments with movement instructions for each obstacle.

Endpoint:
    POST http://<PC_IP>:5001/pathfinding

Usage:
    algo = AlgoInterface(host="<PC_IP>")

    obstacles = [
        {
            "image_id": 1,
            "direction": "WEST",
            "south_west": {"x": 10, "y": 10},
            "north_east": {"x": 11, "y": 11},
        },
    ]

    result = algo.request_path(obstacles)
    if result:
        for segment in result["segments"]:
            print(segment["image_id"], segment["instructions"])

Context manager is supported:
    with AlgoInterface(host="<PC_IP>") as algo:
        ...

Standalone test:
    python algo_interface.py --host <PC_IP> --port 5001
"""

import argparse
import json
import sys
import time
from typing import Any, Optional

import requests


DEFAULT_ROBOT = {
    "direction": "NORTH",
    "south_west": {"x": 0, "y": 0},
    "north_east": {"x": 2, "y": 2},
}


class AlgoInterface:
    """
    HTTP client for the pathfinding algorithm service on the PC.

    Sends obstacle data to the algo service and receives back path segments
    with movement instructions for the robot to follow.
    """

    DEFAULT_PORT = 5001
    DEFAULT_TIMEOUT = 30.0

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
        robot: Optional[dict[str, Any]] = None,
        verbose: bool = True,
    ) -> None:
        """
        Args:
            host:    IP address of the PC running the algo service.
            port:    Port the algo service listens on.
            timeout: HTTP request timeout in seconds.
            robot:   Robot configuration dict. Uses DEFAULT_ROBOT if not given.
            verbose: Whether to request verbose path info from the service.
        """
        self._host = host
        self._port = port
        self._timeout = timeout
        self._robot = robot or DEFAULT_ROBOT
        self._verbose = verbose
        self._base_url = f"http://{self._host}:{self._port}"
        self._session: Optional[requests.Session] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """
        Open a persistent HTTP session to the algo service.

        Returns True once the session is ready. The actual TCP connection
        is established lazily on the first request.
        """
        if self._session is not None:
            print("[ALGO] Session already open")
            return True

        self._session = requests.Session()
        self._session.headers.update({"Content-Type": "application/json"})
        print(f"[ALGO] Session ready -> {self._base_url}")
        return True

    def stop(self) -> None:
        """Close the HTTP session."""
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
        robot: Optional[dict[str, Any]] = None,
    ) -> Optional[dict[str, Any]]:
        """
        Request a pathfinding solution for the given obstacles.

        Args:
            obstacles: List of obstacle dicts, each containing:
                       ``image_id``, ``direction``, ``south_west``, ``north_east``.
            robot:     Optional robot config override for this request.
                       Falls back to the instance default if not provided.

        Returns:
            Parsed JSON response with ``segments`` on success, or None on failure.
            Each segment contains ``image_id``, ``cost``, ``instructions``, and ``path``.
        """
        if not self._session:
            print("[ALGO] Not started – call start() first")
            return None

        robot_cfg = robot or self._robot

        def _direction_to_int(self, direction: str) -> int:
            mapping = {
                "NORTH": 0,
                "EAST": 2,
                "SOUTH": 4,
                "WEST": 6
            }
            return mapping.get(direction, 0)
            
        payload = {
            "robot_x": robot_cfg["south_west"]["x"],
            "robot_y": robot_cfg["south_west"]["y"],
            "robot_dir": self._direction_to_int(robot_cfg["direction"]),
            "retrying": "False",
            "obstacles": [
                {
                    "x": ob["south_west"]["x"],
                    "y": ob["south_west"]["y"],
                    "d": self._direction_to_int(ob["direction"]),
                    "obstacleNumber": ob["image_id"]
                }
                for ob in obstacles
            ]
        }

        url = f"{self._base_url}/pathfinding"
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

        result = data.get("data", {})

        commands = result.get("commands", [])
        distance = result.get("distance", 0)
        path = result.get("path", [])

        print(f"[ALGO] Distance: {distance}")
        print(f"[ALGO] Commands: {len(commands)}")
        print(f"[ALGO] Path nodes: {len(path)}")

        return result

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
        {
            "image_id": 1,
            "direction": "WEST",
            "south_west": {"x": 10, "y": 10},
            "north_east": {"x": 11, "y": 11},
        },
    ]

    with AlgoInterface(host=args.host, port=args.port) as algo:
        print(f"Requesting path for {len(sample_obstacles)} obstacle(s)...")
        result = algo.request_path(sample_obstacles)
        if result:
            print("\nFull response:")
            print(json.dumps(result, indent=2))
        else:
            print("\nNo result received.")
