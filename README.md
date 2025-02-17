# ASINDy–MPC for Dual-System VTOL

This repository contains the implementation of **ASINDy–MPC** (Adaptive Sparse Identification of Nonlinear Dynamics + Model Predictive Control) for a dual-system VTOL flying wing (rotor-based hover + fixed-wing forward flight). The code demonstrates:

- **Data-driven modeling** using [SINDy](https://doi.org/10.1016/j.ifacol.2015.06.080) to identify a sparse representation of the system dynamics.
- **MPC** (Model Predictive Control) formulation that uses the SINDy model for predictions.
- **Lyapunov-based modifications** to the MPC cost function to ensure **Input-to-State Stability (ISS)** of the closed-loop system, even in the presence of bounded disturbances.

## Repository Contents

1. **`/src`**  
   - Main Python/C++ code for the ASINDy–MPC loop.  
   - Implementation of the SINDy algorithm (e.g., using a library or custom routines).
   - MPC solver / optimization code (e.g., CasADi, OSQP, or another solver).

2. **`/docs`**  
   - Additional theoretical notes, including a proof appendix showing how the Lyapunov function and dissipation inequality imply ISS.  
   - Explanation of the spiral motion example (if relevant).

3. **`/examples`**  
   - Example scripts illustrating how to train or update a SINDy model from flight data.  
   - Example of running the MPC with the cost function that includes the Lyapunov term.

## How ASINDy–MPC Works

1. **Sparse Identification (SINDy)**  
   - We collect state and input data from the VTOL system (including both hover and forward-flight dynamics).  
   - We use SINDy to build a sparse model of the form  
     \[
       \dot{x} \;=\; \Upsilon(x,u)\,\Pi,
     \]
     where \(\Upsilon\) is a library of candidate basis functions, and \(\Pi\) is a sparse coefficient matrix.

2. **Model Predictive Control**  
   - An MPC problem is formulated over a prediction horizon, using the SINDy-identified model for state predictions.  
   - We compute an optimal control \(u(\cdot)\) that stabilizes the system subject to constraints (e.g., trust region, actuator limits).

3. **Lyapunov Term in the Cost**  
   - We incorporate a **Lyapunov function** \(V\) into the cost function. In the continuous-time form:
     \[
       J \;=\; \int_{0}^{T}\!\Bigl[\ell\bigl(x(t),u(t)\bigr)
         \;+\;
         \lambda\,V\bigl(x(t)\bigr)\Bigr]\mathrm{d}t
       \;+\;
       V\bigl(x(T)\bigr),
     \]
     where \(\ell\) is the original stage cost (e.g., tracking errors), and \(\lambda\) is a positive weight.  
   - Penalizing large \(V(x)\) at each stage (and at the final time) **ensures** that the controller drives the system toward regions of smaller Lyapunov function, thereby **promoting input-to-state stability (ISS)**.

4. **Closed-Loop ISS Guarantee**  
   - Based on standard **dissipation inequalities** and **ISS theorems**, we can rigorously show that the state remains bounded in the presence of bounded disturbances.  
   - The code and proofs in [`docs/Appendix_ISS`](./docs/Appendix_ISS.md) detail these arguments.

## Key Features

- **Rotor + Fixed-Wing**: The code supports a “dual-system” approach (e.g., for a tilt-rotor or tailsitter).  
- **Adaptive or Real-Time SINDy**: Potential for real-time updating of \(\Pi\) if new data shows changing dynamics.  
- **MPC Implementation**: Either discrete or continuous forms are provided, with an interface to multiple solvers.  
- **Example: Spiral Motion**: Demonstrates how the radial coordinate can be stabilized using the Lyapunov-based cost, leading to convergence even under swirling flight paths.

## Getting Started

1. **Dependencies**  
   - Python 3.8+ (or C++17 if using the C++ version)  
   - SINDy library (or custom code)  
   - An MPC solver (CasADi, OSQP, ACADO, etc.)  
   - (Optional) Tools for simulation or hardware integration.

2. **Running the Example**  
   ```bash
   cd examples
   python run_asindy_mpc_spiral.py
