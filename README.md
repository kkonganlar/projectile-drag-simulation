# Projectile Motion with Air Resistance (RK4 Simulation)

Numerical simulation of two-dimensional projectile motion with quadratic air resistance
was performed using the fourth-order Runge-Kutta (RK4) method.

## Project Overview

This project uses a more realistic physical model that incorporates air resistance instead of the ideal projectile motion.
Air resistance is modelled as proportional to the square of velocity, and the equations of motion were solved using numerical methods.

Different shapes are represented by different drag coefficients.
With all other initial conditions kept constant, the effect of the drag coefficient on motion was compared.

## Physical Model

The object's motion was modeled using Newton's second law.
There are two main forces acting on the object: gravitational force and drag force.

The drag force was taken as proportional to the square of the object's velocity and assumed to act in the opposite direction of motion.
The equations of motion obtained under these assumptions consist of nonlinear differential equations.

## Numerical Method

Since the equations of motion involving air resistance are not linear, they cannot be solved analytically.
Therefore, a numerical integration method was used to solve the equations.

The fourth-order Runge–Kutta method was used in this project.
This method provides higher accuracy and stability compared to simple first-order methods.

## Simulation Parameters

All simulations were performed using the same initial conditions:
- Mass: 0.43 kg
- Initial velocity: 35 m/s
- Launch angle: 20°
- Cross-sectional area: 0.038 m²
- Air density: 1.225 kg/m³
- Gravitational acceleration: 9.8 m/s²
- Time step (dt): 0.0001 s

Only the drag coefficient was varied to represent different object shapes.

## Objects Simulated

The following object shapes were simulated:
- Sphere (Cd = 0.47)
- Cylinder (Cd = 0.82)
- Cube (Cd = 1.05)
- Flat Plate (Cd = 1.28)
- Cone (Cd = 0.50)

## Results
The simulation outputs include:

- Total flight time
- Time to reach maximum height
- Maximum height
- Horizontal range
- Impact speed

Numerical results and detailed comparisons are provided in the accompanying project report.

## Report

A detailed description of the physical model, numerical method, and simulation results
can be found in the project report:

`Effect of Drag Coefficient on Projectile Motion.pdf`

## How to Run

1. Clone the repository
2. Make sure Python and NumPy are installed
3. Install required dependencies:

```bash
pip install -r requirements.txt
```
4. Run the simulation script:

```
python projectile_with_drag.py
```

## Author

Kaan Kemal Onganlar
Date: 20-01-2026