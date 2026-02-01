# Service

This module contains a REST API microservice that is responsible for the car's pathfinding and image recognition/stitching.
It exports 

## Building/Running

Download Python 3.12 from https://www.python.org/downloads/release/python-3120/

The module is created using Python 3.12 and `flask-openapi3`. It uses `pipenv` to manage dependencies. The following 
instructions assumes you're already in the service directory.

Else, change to service directory:
```shell
cd service
```

To install pipenv:
```shell
pip install pipenv
```

To install dependencies:
```shell
pipenv install
```


To switch to virtual environment:
```shell
pipenv shell
```

To run the server:
```shell
python app.py
```

The OpenAPI definitions 
can be downloaded directly at http://localhost:5000/openapi/openapi.json (useful if you're planning to generate an OpenAPI client).

Sample request:
POST http://localhost:5001/pathfinding
In Header:  
```
Content-Type: application/json
```
Request body:
```
{
  "robot": {
    "direction": "EAST",
    "south_west": { "x": 0, "y": 0 },
    "north_east": { "x": 1, "y": 1 }
  },
  "obstacles": [
    {
      "image_id": 1,
      "direction": "WEST",
      "south_west": { "x": 10, "y": 10 },
      "north_east": { "x": 11, "y": 11 }
    }
  ],
  "verbose": true
}
```
Response: 
```
{
    "segments": [
        {
            "cost": 25,
            "image_id": 1,
            "instructions": [
                {
                    "amount": 50,
                    "move": "FORWARD"
                },
                "FORWARD_LEFT",
                {
                    "amount": 100,
                    "move": "FORWARD"
                },
                "BACKWARD_LEFT",
                "CAPTURE_IMAGE"
            ],
            "path": [
                {
                    "direction": "EAST",
                    "x": 2,
                    "y": 1
                },
                {
                    "direction": "EAST",
                    "x": 3,
                    "y": 1
                },
                {
                    "direction": "EAST",
                    "x": 4,
                    "y": 1
                },
                {
                    "direction": "EAST",
                    "x": 5,
                    "y": 1
                },
                {
                    "direction": "EAST",
                    "x": 6,
                    "y": 1
                },
                {
                    "direction": "NORTH",
                    "x": 5,
                    "y": 1
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 4
                },
                {
                    "direction": "NORTH",
                    "x": 6,
                    "y": 2
                },
                {
                    "direction": "NORTH",
                    "x": 7,
                    "y": 3
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 5
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 6
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 7
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 8
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 9
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 10
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 11
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 12
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 13
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 14
                },
                {
                    "direction": "NORTH",
                    "x": 8,
                    "y": 15
                },
                {
                    "direction": "EAST",
                    "x": 5,
                    "y": 11
                },
                {
                    "direction": "EAST",
                    "x": 8,
                    "y": 14
                },
                {
                    "direction": "EAST",
                    "x": 6,
                    "y": 12
                },
                {
                    "direction": "EAST",
                    "x": 7,
                    "y": 13
                },
                {
                    "direction": "EAST",
                    "x": 6,
                    "y": 11
                }
            ]
        }
    ]
}
```