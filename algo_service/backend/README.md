# Task 1 Algorithm Service (Backend)

Pathfinding API for Task 1. See parent README for arena conventions and API details.

## Install dependencies

With **pip** (from repo root):

```bash
pip install -r algo_service/backend/requirements.txt
```

With **uv**:

```bash
uv venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
uv pip install -r algo_service/backend/requirements.txt
```

## Run

From **repository root**:

```bash
python -m algo_service.backend.app
```

Runs on `0.0.0.0:15001` by default (`PORT` env to override).
