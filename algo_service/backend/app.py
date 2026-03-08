"""
Task 1 Algorithm Service – Pathfinding API (backend).

POST /pathfinding
  Body: { "robot": {...}, "obstacles": [...], "verbose": bool }
  Response: { "segments": [ { "image_id", "instructions", "cost", "path" }, ... ] }

Run from repo root: python -m algo_service.backend.app
Or from backend/:     python app.py
"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path

# When run as python app.py from backend/, make backend importable via algo_service in path
if __name__ == "__main__":
    _backend_dir = Path(__file__).resolve().parent
    _algo_service_dir = _backend_dir.parent
    if str(_algo_service_dir) not in sys.path:
        sys.path.insert(0, str(_algo_service_dir))

from flask import Flask, request, jsonify
from flask_cors import CORS

try:
    from .solver import build_segments
except ImportError:
    from backend.solver import build_segments

app = Flask(__name__)
CORS(app)
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@app.route("/pathfinding", methods=["POST"])
def pathfinding():
    """
    Compute path segments for Task 1: navigate to each obstacle and capture image.
    """
    try:
        data = request.get_json(force=True, silent=True)
    except Exception:
        return jsonify({"error": "Invalid JSON"}), 400

    if not data:
        return jsonify({"error": "Missing JSON body"}), 400

    robot = data.get("robot")
    obstacles = data.get("obstacles", [])

    if not robot:
        return jsonify({"error": "Missing 'robot' in body"}), 400

    if not isinstance(obstacles, list):
        return jsonify({"error": "'obstacles' must be a list"}), 400

    segments = build_segments(robot, obstacles)
    logger.info("pathfinding: %d obstacles -> %d segments", len(obstacles), len(segments))

    return jsonify({"segments": segments})


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"})


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 15001))
    app.run(host="0.0.0.0", port=port, debug=True)
