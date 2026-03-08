"""
Task 1 Algorithm Service – Pathfinding API

POST /pathfinding
  Body: { "robot": {...}, "obstacles": [...], "verbose": bool }
  Response: { "segments": [ { "image_id", "instructions", "cost", "path" }, ... ] }

Run: python -m algo_service.app
     or: flask --app algo_service.app run -p 5001
"""

from __future__ import annotations

import logging

from flask import Flask, request, jsonify

from .solver import build_segments

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@app.route("/pathfinding", methods=["POST"])
def pathfinding():
    """
    Compute path segments for Task 1: navigate to each obstacle and capture image.

    Expected JSON body:
      robot: { "direction": "NORTH", "south_west": {"x": 0, "y": 0}, "north_east": {"x": 3, "y": 3} }
      obstacles: [ { "image_id": 1, "direction": "NORTH", "south_west": {"x": 12, "y": 12}, "north_east": {"x": 13, "y": 13} }, ... ]
      verbose: optional bool

    Returns:
      { "segments": [ { "image_id", "instructions", "cost", "path" }, ... ] }
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
    import os
    port = int(os.environ.get("PORT", 15001))
    app.run(host="0.0.0.0", port=port, debug=True)
