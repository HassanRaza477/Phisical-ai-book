---
id: ch02-ros2-fundamentals
title: "Chapter 2: Fundamentals of ROS 2"
stage: prompt
date: 2025-12-04
surface: content/chapters/02-ros2-fundamentals.md
model: gemini-1.5-pro
feature: textbook-writing
branch: feat/chapter-02
user: AI-Writer
command: "specifyplus write"
labels: [chapter, p1]
links:
  spec: humanoid-robotics-book/spec.md
files:
  - humanoid-robotics-book/plan.md
---

## Prompt

You are an expert technical writer and robotics educator. Your task is to write Chapter 2 of the textbook "Physical AI & Humanoid Robotics," titled "Fundamentals of ROS 2."

**Reference**: `humanoid-robotics-book/plan.md`

**Instructions**:
1.  **Objective**: Equip the reader with a solid, practical understanding of the ROS 2 framework. This chapter should be very hands-on.
2.  **Cover all topics from the plan**:
    *   **Core Concepts**: Thoroughly explain Nodes, Topics, Services, Actions, and Parameters. For each concept, provide a simple, clear analogy and a working code example (in both C++ and Python).
    *   **The ROS 2 Transform System (`tf2`)**: Explain why managing coordinate frames is critical in robotics. Show how to broadcast and listen for transforms. Include a visual example using `rviz2`.
    *   **Workspace and Packages**: Guide the reader through creating a new workspace and a simple C++/Python package.
    *   **Debugging Tools**: Provide a practical guide to using `rqt` (especially `rqt_graph`), `rviz2`, and essential `ros2` CLI commands (`node list`, `topic echo`, etc.).
    *   **The `launch` System**: Explain the importance of launch files for starting complex systems. Show how to write a launch file that starts multiple nodes and configures them.
3.  **Code Examples**: All code must be complete, runnable, and well-commented. Provide instructions on how to compile and run each example.
4.  **Tone**: Clear, direct, and practical.
5.  **Output**: Produce a markdown file with the full chapter content and associated code snippets.
