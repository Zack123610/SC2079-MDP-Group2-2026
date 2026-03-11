# Service

This module contains a REST API microservice responsible for the car's pathfinding.
It uses Python 3.12, `flask-openapi3`, and an A* + TSP solver to compute optimal
obstacle-visiting paths.

## Building/Running

### Prerequisites

- Python 3.12 ([download](https://www.python.org/downloads/release/python-3120/))
- [uv](https://docs.astral.sh/uv/getting-started/installation/)

### Setup

Change to the service directory:
```shell
cd service
```

Create a virtual environment and install dependencies:
```shell
uv venv --python 3.12
source .venv/bin/activate
uv pip install flask-cors jinja2 marshmallow marshmallow-enum numpy pydantic "flask-openapi3[swagger]" python-tsp
```

### Run

```shell
python app.py
```

The server starts on `http://0.0.0.0:15001` by default.

The OpenAPI spec is available at http://localhost:15001/openapi/openapi.json.

## API

### `POST /pathfinding/`

Request body:
```json
{
  "robot_x": 1,
  "robot_y": 1,
  "robot_dir": 0,
  "retrying": false,
  "obstacles": [
    {"x": 0, "y": 15, "d": 2, "obstacleNumber": 1},
    {"x": 17, "y": 19, "d": 4, "obstacleNumber": 2},
    {"x": 12, "y": 13, "d": 0, "obstacleNumber": 3},
    {"x": 19, "y": 8, "d": 6, "obstacleNumber": 4},
    {"x": 14, "y": 1, "d": 0, "obstacleNumber": 5},
    {"x": 7, "y": 5, "d": 0, "obstacleNumber": 6},
    {"x": 5, "y": 11, "d": 0, "obstacleNumber": 7}
  ]
}
```

- `robot_dir`: `0` = North, `2` = East, `4` = South, `6` = West
- `d` (obstacle direction): same encoding as `robot_dir`

Response:
```json
{
  "data": {
    "distance": 46.0,
    "path": [...],
    "commands": [
      "FW60", "FR00", "FW90", "FW10", "SNAP4_C",
      "BW10", "FR00", "BW10", "SNAP5_C",
      "BR00", "FW30", "FL00", "FW20", "SNAP2_C",
      "FW10", "FL00", "FW50", "BR00", "BW10", "SNAP3_C",
      "FW10", "FL00", "BW70", "BL00", "FW10", "SNAP7_C",
      "FL00", "BW10", "FR00", "FW20", "SNAP6_R",
      "BW20", "BL00", "FW40", "SNAP1_C",
      "FN"
    ]
  },
  "error": null
}
```

Command format:
| Command | Meaning |
|---------|---------|
| `FWxx` | Forward xx units |
| `BWxx` | Backward xx units |
| `FR00` | Forward Right turn |
| `FL00` | Forward Left turn |
| `BR00` | Backward Right turn |
| `BL00` | Backward Left turn |
| `SNAPn_X` | Capture image for obstacle n (`C` = center, `L` = left, `R` = right) |
| `FN` | End of commands |
