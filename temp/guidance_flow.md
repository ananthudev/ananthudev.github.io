# Guidance Algorithm Flow Diagram

## High-Level System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    GUIDANCE SYSTEM INPUTS                      │
├─────────────────────────────────────────────────────────────────┤
│ Launch Coordinates: (lat_O, lon_O, alt_O)                     │
│ Target Coordinates: (lat_T, lon_T, alt_T)                     │
│ Initial Projectile States: (x_0_eci, y_0_eci, z_0_eci,       │
│                           vx_0_eci, vy_0_eci, vz_0_eci)      │
│ Desired Impact Angles: (theta_f, psi_f)                       │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    CONSTANTS PHASE                             │
├─────────────────────────────────────────────────────────────────┤
│ 1. Physical Constants: g, rho                                 │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    INITIALIZATION PHASE                        │
├─────────────────────────────────────────────────────────────────┤
│ 1. Set Parameters: N, m_c, C_d, dia, S_ref                    │
│ 2. Guidance Parameters: a, b, m, n, d, K, sigma_max           │
│ 3. Simulation Parameters: dt, max_time, min_velocity           │
│ 4. Convert Target to ECI: geodeticToECI()                     │
│ 5. Transform Initial States to Local Frame                     │
│ 6. Calculate Desired Impact Direction Vector                   │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    MAIN GUIDANCE LOOP                          │
├─────────────────────────────────────────────────────────────────┤
│ WHILE (t < max_time AND r ≤ r_prev AND velocity > min_velocity)│
│                                                                 │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              STATE CALCULATIONS                            │ │
│ │ • Current Position: R_m = [x, y, z]                      │ │
│ │ • Current Velocity: V_m = [vx, vy, vz]                   │ │
│ │ • Velocity Magnitude: v = norm(V_m)                       │ │
│ │ • Velocity Unit Vector: e_m = V_m / v                     │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                    │                           │
│                                    ▼                           │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              TARGET VECTOR CALCULATIONS                   │ │
│ │ • Vector to Target: R = R_t_0 - R_m                      │ │
│ │ • Distance to Target: r = norm(R)                        │ │
│ │ • Target Unit Vector: e_R = R / r                        │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                    │                           │
│                                    ▼                           │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              GUIDANCE LAW CALCULATIONS                     │ │
│ │ • Look Angle: sigma = acos(dot(e_m, e_R))                │ │
│ │                                                                 │
│ │ IF (r ≤ r_loop3_start)                                     │ │
│ │   • Terminal Phase: A_M = [0, 0, 0]                      │ │
│ │ ELSE                                                        │ │
│ │   • Guided Phase: Complex guidance calculations            │ │
│ │     ├── Angular Velocity: omega_L = cross(V_m, R) / r²   │ │
│ │     ├── Quaternion Calculations                            │ │
│ │     ├── Rotation Matrix: L_q                              │ │
│ │     ├── Time-to-Go: t_go = r / (v * cos(sigma))          │ │
│ │     └── Final Acceleration: A_M = f(guidance_law)        │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                    │                           │
│                                    ▼                           │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              FORCE CALCULATIONS                            │ │
│ │ • Drag Force: D = 0.5 * rho * v² * C_d * S_ref          │ │
│ │ • Drag Acceleration: a_D = D / m_c                        │ │
│ │ • Total Acceleration: a_total = A_M - a_D - g            │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                    │                           │
│                                    ▼                           │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              STATE INTEGRATION                             │ │
│ │ • Update Velocity: v += a_total * dt                      │ │
│ │ • Update Position: r += v * dt                            │ │
│ │ • Transform Back to ECI Frame                             │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                    │                           │
│                                    ▼                           │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              TERMINATION CHECK                             │ │
│ │ • Check if r > r_prev (target impact)                     │ │
│ │ • Check if t ≥ max_time (time exceeded)                   │ │
│ │ • Check if velocity ≤ min_velocity (velocity too low)     │ │
│ │ • Check if z < 0 (altitude negative)                      │ │
│ └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    SYSTEM OUTPUTS                              │
├─────────────────────────────────────────────────────────────────┤
│ Final Projectile Position: R_m_ECI_final                     │
│ Final Projectile Velocity: V_m_ECI_final                     │
│ Impact Status and Statistics                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Detailed Function Dependencies

```
┌─────────────────────────────────────────────────────────────────┐
│                    FUNCTION HIERARCHY                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ Level 1: Core Mathematical Functions                           │
│ ├── vector_add(), vector_subtract(), vector_scale()           │
│ ├── vector_magnitude(), vector_normalize()                    │
│ ├── vector_dot_product(), vector_cross_product()              │
│ ├── cos(), sin(), asin(), atan2(), acos()                    │
│ ├── sqrt(), abs(), sign()                                     │
│ └── sigmoid() (custom function)                               │
│                                                                 │
│ Level 2: Coordinate Transformations                            │
│ ├── geodeticToECI(), eciToGeodetic()                         │
│ ├── ECI_to_Local(), Local_to_ECI()                           │
│ ├── ECI_to_Local_vel(), Local_to_ECI_vel()                   │
│ ├── body_to_local(), local_to_body()                          │
│ └── computeRotationMatrix()                                   │
│                                                                 │
│ Level 3: Quaternion Operations                                │
│ ├── quaternion_from_axis_angle()                              │
│ ├── quaternion_multiply()                                     │
│ ├── quaternion_rotate_vector()                                │
│ └── quaternion_to_matrix()                                    │
│                                                                 │
│ Level 4: Guidance Algorithm                                   │
│ └── onboard_guidance_algorithm_2() (main function)            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Data Flow Diagram

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   INPUT     │───▶│  INITIALIZE │───▶│   MAIN      │
│ PARAMETERS  │    │   PHASE     │    │   LOOP      │
└─────────────┘    └─────────────┘    └─────────────┘
                                              │
                                              ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   OUTPUT    │◀───│  TERMINATE  │◀───│  UPDATE     │
│  RESULTS    │    │   CHECK     │    │  STATES     │
└─────────────┘    └─────────────┘    └─────────────┘
                                              │
                                              ▼
                                    ┌─────────────┐
                                    │  GUIDANCE   │
                                    │  CALCULATE  │
                                    └─────────────┘
                                              │
                                              ▼
                                    ┌─────────────┐
                                    │  FORCE      │
                                    │  CALCULATE  │
                                    └─────────────┘
```

## Memory Usage Pattern

```
┌─────────────────────────────────────────────────────────────────┐
│                    MEMORY LAYOUT                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ Physical Constants: g, rho                                     │
│ Initialization Parameters: N, m_c, C_d, dia, S_ref, a, b, m,   │
│ n, d, K, sigma_max, dt, max_time, min_velocity                 │
│ State Variables (Read-Write):                                  │
│ ├── Position: x, y, z (3 floats)                              │
│ ├── Velocity: vx, vy, vz (3 floats)                           │
│ ├── Time: t (1 float)                                          │
│ └── Distance: r, r_prev (2 floats)                            │
│ Temporary Variables (Per Iteration):                           │
│ ├── Vectors: R_m, V_m, R, e_m, e_R (15 floats)               │
│ ├── Guidance: A_M, omega_L, k_L (9 floats)                   │
│ ├── Quaternions: w, x_q, y_q, z_q (4 floats)                 │
│ ├── Forces: a_D, a_total (6 floats)                           │
│ └── Others: sigma, theta, psi, etc. (10 floats)               │
│                                                                 │
│ Total Estimated Memory: ~150 floats (~600 bytes)               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Computational Complexity

```
┌─────────────────────────────────────────────────────────────────┐
│                    COMPUTATIONAL ANALYSIS                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ Per Iteration Operations:                                       │
│ ├── Vector Operations: ~20 operations                          │
│ ├── Trigonometric Functions: ~10 calls                         │
│ ├── Coordinate Transformations: ~2 calls                       │
│ ├── Quaternion Operations: ~5 operations                       │
│ └── Basic Arithmetic: ~50 operations                           │
│                                                                 │
│ Critical Path (Most Expensive):                                │
│ ├── Coordinate Transformations (ECI ↔ Local)                   │
│ ├── Quaternion Calculations                                    │
│ ├── Matrix Multiplications                                     │
│ └── Trigonometric Functions                                    │
│                                                                 │
│ Estimated CPU Time per Iteration:                              │
│ ├── Low-end MCU: 5-10ms                                        │
│ ├── Mid-range MCU: 2-5ms                                       │
│ └── High-end MCU: 0.5-2ms                                      │
│                                                                 │
│ Loop Frequency: 100 Hz (dt = 0.01s)                           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```
