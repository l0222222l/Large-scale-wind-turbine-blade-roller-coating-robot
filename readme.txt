# Adaptive Impedance Control & Parameter Identification Simulations

[cite_start]This repository contains MATLAB simulation codes for robotic contact tasks, specifically focusing on applications like automated roller coating for large-scale wind turbine blades[cite: 1, 2].

The project validates two core research areas:
1.  [cite_start]**Environment Stiffness Identification:** Comparing **IRLS (Iteratively Reweighted Least Squares)** against traditional **RLS (Recursive Least Squares)** to handle stiffness mutations and fluctuations[cite: 2, 5, 14].
2.  [cite_start]**Adaptive Impedance Control:** Implementing **DT-MPC (Discrete-Time Model Predictive Control)** to dynamically optimize impedance parameters (Inertia $m$, Damping $b$, Stiffness $k$) based on real-time contact force[cite: 46, 49].

## 🛠️ Prerequisites

To run these simulations, you need **MATLAB** (Recommended R2018b or later) with the following toolbox installed:

* [cite_start]**Optimization Toolbox** (Required for the `quadprog` function used in the MPC simulations)[cite: 48].

## 📂 File Structure

The codebase is organized into three main categories: Parameter Identification, DT-MPC Control, and Basic Impedance Analysis.

### 1. Parameter Identification
These scripts analyze the performance of IRLS versus RLS under different environmental stiffness conditions.

| File Name | Description | Related Scenario |
| :--- | :--- | :--- |
| `Parameter_identification001.m` | [cite_start]**Stiffness Fluctuation.** Compares tracking performance of IRLS vs. RLS when environment stiffness fluctuates sinusoidally[cite: 13, 14]. | Dynamic Stiffness |
| `Parameter_identification002.m` | [cite_start]**Stiffness Mutation.** Compares convergence speed when stiffness undergoes a step change (e.g., 5N/mm to 10N/mm)[cite: 4, 5]. | Sudden Change |
| `Parameter_identification004.m` | [cite_start]**IRLS vs. Impedance Control.** Contrasts force tracking errors between IRLS-based control and fixed-parameter impedance control under stiffness fluctuations[cite: 16]. | Control Comparison |

### 2. DT-MPC Variable Impedance Control
These scripts implement Discrete-Time Model Predictive Control to optimize impedance parameters online for various surface profiles.

| File Name | Description | Environment |
| :--- | :--- | :--- |
| `MPC.m` | [cite_start]**Planar Surface.** Baseline comparison between MPC-optimized control and traditional impedance control with a fixed reference position ($m$, $b$, $k$ optimization)[cite: 47, 49]. | Flat |
| `MPC001.m` | [cite_start]**Sinusoidal Surface.** Validates algorithm adaptability when the reference position follows a sine wave[cite: 55, 57]. | Curved |
| `MPC002.m` | [cite_start]**Irregular Surface.** Simulates a complex environment (Sine + Cosine + Decay) to test robustness against irregular disturbances[cite: 63, 65]. | Complex/Rough |

### 3. Basic Impedance Characteristics
Fundamental simulations to understand how system parameters affect contact dynamics.

| File Name | Description | Focus |
| :--- | :--- | :--- |
| `ImpedanceControl001.m` | [cite_start]**Parameter Influence.** Analyzes how varying Inertia ($m$), Damping ($b$), and Stiffness ($k$) affects overshoot and settling time[cite: 27, 29]. | Controller Tuning |
| `ImpedanceControl003.m` | [cite_start]**Environment Influence.** Demonstrates how different environmental stiffnesses (Low, Medium, High) impact contact force oscillation[cite: 22, 25]. | Environment |

## 🚀 Getting Started

1.  **Clone the repository** to your local machine.
2.  Open **MATLAB** and navigate to the repository folder.
3.  **Run Parameter Identification:**
    * Execute `Parameter_identification002.m` to visualize how IRLS handles sudden stiffness changes compared to RLS.
4.  **Run MPC Simulation:**
    * Execute `MPC.m` to see the adaptive control in action.
    * *Note:* This script uses `quadprog` for quadratic programming. Ensure the Optimization Toolbox is installed. The simulation may take a few seconds to compute.

## 📊 Key Results

* **Stiffness Identification:** IRLS demonstrates superior convergence and lower error rates compared to RLS. [cite_start]It avoids the saturation effects seen in RLS during continuous iterations and adapts faster to stiffness mutations[cite: 5, 21].
* [cite_start]**Vibration Suppression:** In irregular surface simulations, the DT-MPC approach significantly reduced force overshoot (e.g., from **70%** in traditional control to **31%** with MPC) and accelerated the settling time[cite: 66, 71].

## 📝 Reference

[cite_start]This code is part of a doctoral research project on robotic force control and adaptive algorithms for dynamic environments[cite: 1].
