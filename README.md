# SC2079-MDP-Group2-2026

Project Repo for SC2079 Multi-Disciplinary Project 2026

## Project Overview

This project is a **mini car robot navigation challenge**. The robot must navigate to multiple obstacles and complete image recognition tasks by identifying numbers displayed on each obstacle.

## System Architecture

```
┌─────────────────┐   Bluetooth   ┌─────────────────┐
│  Android Tablet │◄────────────►│                 │
│  (User Input)   │              │                 │
└─────────────────┘              │  Raspberry Pi   │
                                 │  (Comm Hub +    │
┌─────────────────┐    Serial    │   Camera)       │
│   STM32 Board   │◄────────────►│                 │
│ (Motor Control) │              │                 │
└─────────────────┘              └────────┬────────┘
                                          │
                                          │ WiFi Video Stream
                                          ▼
                                 ┌─────────────────┐
                                 │       PC        │
                                 │ (YOLO Inference │
                                 │  + Pathfinding) │
                                 └─────────────────┘
```
