# AutonomousTest

This project simulates the trajectory of an autonomous vehicle, generating the sensor signals it would receive and processing them using various localization algorithms like FastSLAM and EKF (Extended Kalman Filter). The system visually compares real vs. simulated vehicle positions to assess the performance of each algorithm.

## Features
- **Vehicle Motion Simulation**: Generates real-time sensor data for localization.
- **FastSLAM Algorithm** (`fastslam.py`): Implements the FastSLAM algorithm to estimate the vehicle's position based on particle filtering, landmarks, and sensor data.
- **Visualization**: Displays real and simulated vehicle positions for comparison.

## How to Use
1. **Clone the Repository**.
2. **Run the Scripts** using Python (`main.py`) to simulate and visualize the vehicle’s path.
