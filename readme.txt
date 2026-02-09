Here is the English version of the `README.md` file, tailored for your GitHub repository.

---

# Adaptive Impedance Control & Parameter Identification Simulations

This repository contains MATLAB simulation codes focused on robotic contact tasks, specifically designed for applications such as the automated roller coating of large-scale wind turbine blades.

The project validates two core research areas:

1. **Environment Stiffness Identification:** Comparing **IRLS (Iteratively Reweighted Least Squares)** against traditional **RLS** to handle stiffness mutations and fluctuations.
2. **Adaptive Impedance Control:** Implementing **DT-MPC (Discrete-Time Model Predictive Control)** to dynamically optimize impedance parameters (Inertia , Damping , Stiffness ) based on real-time contact force.

## 🛠️ Prerequisites

To run these simulations, you need **MATLAB** (Recommended R2018b or later) with the following toolbox installed:

* **Optimization Toolbox** (Required for the `quadprog` function used in the MPC simulations).

## 📂 File Structure

The codebase is organized into three main categories: Parameter Identification, DT-MPC Control, and Basic Impedance Analysis.

### 1. Parameter Identification

These scripts analyze the performance of IRLS versus RLS under different environmental stiffness conditions.

| File Name | Description | Scenario |
| --- | --- | --- |
| `Parameter_identification001.m` | **Stiffness Fluctuation.** Compares tracking performance of IRLS vs. RLS when environment stiffness fluctuates sinusoidally. | Dynamic Stiffness |
| `Parameter_identification002.m` | **Stiffness Mutation.** Compares convergence speed when stiffness undergoes a step change (e.g., 5N/mm to 10N/mm). | Sudden Change |
| `Parameter_identification004.m` | **IRLS vs. Impedance Control.** Contrasts force tracking errors between IRLS-based control and fixed-parameter impedance control under stiffness fluctuations. | Control Comparison |

### 2. DT-MPC Variable Impedance Control

These scripts implement Discrete-Time Model Predictive Control to optimize impedance parameters online for various surface profiles.

| File Name | Description | Environment |
| --- | --- | --- |
| `MPC.m` | **Planar Surface.** Baseline comparison between MPC-optimized control and traditional impedance control with a fixed reference position. | Flat |
| `MPC001.m` | **Sinusoidal Surface.** Validates algorithm adaptability when the reference position follows a sine wave. | Curved |
| `MPC002.m` | **Irregular Surface.** Simulates a complex environment (Sine + Cosine + Decay) to test robustness against irregular disturbances. | Complex/Rough |

### 3. Basic Impedance Characteristics

Fundamental simulations to understand how system parameters affect contact dynamics.

| File Name | Description | Focus |
| --- | --- | --- |
| `ImpedanceControl001.m` | **Parameter Influence.** Analyzes how varying Inertia (), Damping (), and Stiffness () affects overshoot and settling time. | Controller Tuning |
| `ImpedanceControl003.m` | **Environment Influence.** Demonstrates how different environmental stiffnesses (Low, Medium, High) impact contact force oscillation. | Environment |

## 🚀 Getting Started

1. **Clone the repository** to your local machine.
2. Open **MATLAB** and navigate to the repository folder.
3. **Run Parameter Identification:**
* Execute `Parameter_identification002.m` to visualize how IRLS handles sudden stiffness changes compared to RLS.


4. **Run MPC Simulation:**
* Execute `MPC.m` to see the adaptive control in action.
* *Note:* This script uses `quadprog` for quadratic programming. Ensure the Optimization Toolbox is installed. The simulation may take a few seconds to compute.



## 📊 Key Results

* 
**Stiffness Identification:** IRLS demonstrates superior convergence and lower error rates compared to RLS, particularly avoiding the saturation effects seen in RLS during continuous iterations.


* 
**Vibration Suppression:** In irregular surface simulations, the DT-MPC approach significantly reduced force overshoot (e.g., from **70%** in traditional control to **31%** with MPC) and accelerated the settling time.



## 📝 Reference

This code is part of a doctoral research project on robotic force control and adaptive algorithms for dynamic environments. For detailed theoretical analysis, please refer to the associated documentation.

---

*Generated based on the provided MATLAB scripts and research documentation.*
