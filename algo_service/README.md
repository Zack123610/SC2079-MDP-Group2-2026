# Task 1 Algorithm Service

Pathfinding for **Task 1**: navigate a 3×3 robot around 1×1 obstacles in a 20×20 arena, reach a view position (2 units from each obstacle face), and trigger `CAPTURE_IMAGE` without colliding.

## Structure

| Directory | Description |
|-----------|-------------|
| **backend/** | Python Flask API: pathfinding logic, `POST /pathfinding`, `GET /health`. Run on port 15001 by default. |
| **frontend/** | React + Vite UI to configure robot/obstacles and request paths. Proxies `/api` to the backend in dev. |

## Arena and conventions

- **Arena**: 20×20 units (grid 0..19).
- **Robot**: 3×3 units; position = south-west corner.
- **Obstacle**: 1×1 unit; one face has the valid image. **Capture distance**: 2 units from the obstacle face (camera works best at this distance).
- **Unit → STM32**: 1 unit = 10 cm for FORWARD/BACKWARD amounts.

Movements: FORWARD, BACKWARD (with amount in cm), FORWARD_LEFT, FORWARD_RIGHT, BACKWARD_LEFT, BACKWARD_RIGHT, CAPTURE_IMAGE. No stationary turns.

## Run

### Backend (from repo root)

```bash
pip install -r algo_service/backend/requirements.txt
python -m algo_service.backend.app
```

Listens on `0.0.0.0:15001` (override with `PORT`).

### Frontend (dev)

```bash
cd algo_service/frontend
npm install
npm run dev
```

Open http://localhost:5173. The UI proxies `/api` to `http://localhost:15001`; ensure the backend is running. You can change “API base” in the UI to point to another host (e.g. `http://<PC_IP>:15001`) when testing from another machine.

### Production build

```bash
cd algo_service/frontend
npm run build
```

Serve the `dist/` folder with any static host. Set the API base in the app or via env when building if the backend is on a different origin.

## RPi integration

- **rpi/task1.py** uses `--algo-port 15001` by default and POSTs to the algo backend. Start the backend (e.g. on the PC) and run task1 with the PC’s IP.

## API (backend)

- **POST /pathfinding** — Body: `{ "robot": { "direction", "south_west", "north_east" }, "obstacles": [ { "image_id", "direction", "south_west", "north_east" }, ... ] }`. Returns `{ "segments": [ { "image_id", "instructions", "cost", "path" }, ... ] }`.
- **GET /health** — Returns `{ "status": "ok" }`.

See **backend/README.md** for run details and **backend/** for implementation.
