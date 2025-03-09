# multi-robot-coordination-Rendez-vous-Problem-Target
## Introduction
This project addresses the challenge of multi-robot coordination, specifically focusing on the rendez-vous problem where multiple autonomous robots converge to a static point and then transition to following a dynamic trajectory. The project explores consensus-based control laws through MATLAB simulations, demonstrating how robots can adaptively converge and track moving targets.

## Project Overview
The objectives of this project include:
- **Convergence to a Static Point**: Initially, all robots converge to a predetermined static location using consensus-based control strategies.
- **Following a Moving Target**: Post-convergence, robots follow a moving target along a predefined path (`y = 2x^3`), showcasing their ability to adapt to dynamic changes in their environment.

## Project Structure
- `simulate_robots.m`: MATLAB function for simulating the initial static rendez-vous problem.
- `simulate_robots2.m`: MATLAB function for simulating robot coordination to a moving target following the initial convergence.
- `ars1_report.pdf`: Detailed documentation that outlines the theoretical background, methodologies, simulations, and results.

## Getting Started
To get started with the simulations:
1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourgithubusername/your-repository-name.git
2. **Open MATLAB and navigate to the project directory.**
3. **Run the MATLAB scripts to observe the simulations:**
- For static convergence: simulate_robots
- For dynamic target following:

4. **Simulation Details**
Static Convergence: The first script demonstrates how robots can effectively meet at a single point using distributed control laws.
Dynamic Target Following: The second script extends the scenario to include target tracking, illustrating how robots can adjust their paths to follow a moving target dynamically.
