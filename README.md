# Autonomous Drone Platform (ADP-1.0)

_This project is version 1.0 of an ongoing collaboration between [Lars Gartenberg](https://github.com/larsgart) and [Jerin Abraham](https://github.com/jerinabr)._

The goal of ADP is to construct an autonomous quadcopter from scratch, capable of simultaneous location and mapping (SLAM), collision avoidance, and object detection.

# Hardware/ Software Specifications
List sensors, camera, motors, battery, frame specifications, etc.

All computation is done on-board with the Nvidia Jetson Nano (2GB).

Frame data is captured using the M12 Lens Camera for Raspberry Pi and is preprocessed through Python's OpenCV library.

# Research Papers

[Control of a Quadrotor with Reinforcement Learning](https://arxiv.org/abs/1707.05110)

[Deep Convolutional Neural Network-Based Autonomous Drone Navigation](https://arxiv.org/abs/1905.01657.pdf)

[AutoPilot: Automating Co-Design Space Exploration for Autonomous UAVs](https://arxiv.org/abs/2102.02988.pdf)

[Learning to Fly—a Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control](https://arxiv.org/abs/2103.02142v3.pdf)
