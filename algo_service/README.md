# Task 1 Algorithm Service

Pathfinding service for **Task 1**: navigate a 3×3 robot around 1×1 obstacles in a 20×20 arena, reach a valid view position for each obstacle, and trigger `CAPTURE_IMAGE` to identify the valid image on each obstacle without colliding.

## Arena and Conventions

| Item | Size / Notes |
|------|----------------|
| Arena | 20 × 20 units (grid cells 0..19) |
| Robot | 3 × 3 units; position = south-west corner of the robot |
| Obstacle | 1 × 1 unit; one face has the valid image |
| Capture distance | 2 units from the obstacle face (image recognition works best at this distance) |
| Unit → STM32 | 1 unit = 10 cm for `FORWARD` / `BACKWARD` amounts |

## Supported Movements

- **FORWARD** / **BACKWARD** – amount in cm (e.g. 20 = 2 units).
- **FORWARD_LEFT** – 2 units forward, 3 units left (single arc).
- **FORWARD_RIGHT** – 2 units forward, 3 units right.
- **BACKWARD_LEFT** / **BACKWARD_RIGHT** – same pattern backward.
- **CAPTURE_IMAGE** – issued when the robot is in a valid view position (2 units from the obstacle face).

The service does **not** use stationary turns (LEFT/RIGHT in place); only the above moves. View positions are chosen so the robot is **2 units away** from the obstacle (camera works best at this distance).

## API

### `POST /pathfinding`

**Request (JSON):**

```json
{
  "robot": {
    "direction": "NORTH",
    "south_west": { "x": 0, "y": 0 },
    "north_east": { "x": 3, "y": 3 }
  },
  "obstacles": [
    {
      "image_id": 1,
      "direction": "NORTH",
      "south_west": { "x": 12, "y": 12 },
      "north_east": { "x": 13, "y": 13 }
    }
  ],
  "verbose": true
}
```

- **robot.direction** – Initial heading: `NORTH`, `SOUTH`, `EAST`, `WEST`.
- **robot.south_west** – Start position (south-west corner of the 3×3 robot).
- **obstacles[].image_id** – Identifier for this obstacle (returned in the segment).
- **obstacles[].direction** – Face on which the valid image is (where the robot must view from the opposite side).
- **obstacles[].south_west** / **north_east** – 1×1 obstacle cell (typically `north_east = south_west + 1`).

**Response (JSON):**

```json
{
  "segments": [
    {
      "image_id": 1,
      "instructions": [
        { "move": "FORWARD", "amount": 200 },
        "FORWARD_LEFT",
        "CAPTURE_IMAGE"
      ],
      "cost": 3,
      "path": []
    }
  ]
}
```

Instructions are compatible with `rpi/convert_instructions.py` and `rpi/task1.py`: dicts for FORWARD/BACKWARD (with `amount` in cm), strings for arc turns and `CAPTURE_IMAGE`.

## Run

From the **repository root**:

```bash
pip install -r algo_service/requirements.txt
python -m algo_service.app
```

Or from `algo_service/`:

```bash
pip install -r requirements.txt
python app.py
```

Service listens on `0.0.0.0:15001` by default (override with `PORT=5001` if needed). This matches `rpi/task1.py`'s default `--algo-port 15001`.

## RPi Integration

- **rpi/task1.py** sends `ROBOT` and `OBSTACLE` data from Android, then on `BEGIN` POSTs to the algo service (e.g. `--algo-port 15001` if you run the service on 15001).
- The RPi maps each instruction to STM32 commands and Android UI; for `CAPTURE_IMAGE` it runs the detection tracker and sends `TARGET,<obstacle_id>,<target_id>` to Android.

Ensure the algo service port matches `--algo-port` when starting `task1.py`.

## Files

| File | Role |
|------|------|
| `config.py` | Arena size, robot size, unit→cm, **capture distance (2 units)**, arc deltas, directions |
| `grid.py` | Robot/obstacle cells, collision, view/capture positions |
| `pathfinding.py` | A* search, moves (FORWARD/BACKWARD/arc), path → instructions |
| `solver.py` | Builds segments (path to each obstacle + CAPTURE_IMAGE) |
| `app.py` | Flask app: `POST /pathfinding`, `GET /health` |
| `requirements.txt` | Flask dependency |
