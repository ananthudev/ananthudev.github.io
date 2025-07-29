# Code Flow Pictorial Representation

## Main Algorithm Code Flow

```
                    ┌─────────────────────────────────────────────────┐
                    │           FUNCTION ENTRY                       │
                    │   onboard_guidance_algorithm_3()               │
                    │   Input: lat_O, lon_O, alt_O, lat_T, lon_T,   │
                    │   alt_T, x_0_eci, y_0_eci, z_0_eci, vx_0_eci, │
                    │   vy_0_eci, vz_0_eci, theta_f, psi_f          │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           CONSTANTS DEFINITION                 │
                    │   g = 9.8; N = 3; m_c = 47; C_d = 0.2808;    │
                    │   rho = 1.225; dia = 0.155; S_ref = π*dia²/4  │
                    │   a = 0.100001; b = 0.100001; m = 0.90001;    │
                    │   n = 1.30001; d = 7.5; K = 21.00000001;     │
                    │   sigma_max = deg2rad(73); dt = 0.01;         │
                    │   max_simulation_time = 200; min_velocity = 1 │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           TARGET SETUP                         │
                    │   R_t_0_ECI = geodeticToECI(lat_T, lon_T, alt_T) │
                    │   V_t_0_ECI = [0; 0; 0];                      │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           INITIAL PROJECTILE STATES            │
                    │   R_m_ECI = [x_0_eci; y_0_eci; z_0_eci];     │
                    │   V_m_ECI = [vx_0_eci; vy_0_eci; vz_0_eci];  │
                    │   t = 0;                                       │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           INITIAL DISTANCE CALCULATION         │
                    │   R_m_local = ECI_to_Local(R_m_ECI, ...);     │
                    │   R_t_local = ECI_to_Local(R_t_0_ECI, ...);   │
                    │   r = norm(R_t_local - R_m_local);            │
                    │   r_prev = r;                                  │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           DESIRED IMPACT DIRECTION             │
                    │   e_f = [cos(theta_f)*cos(psi_f);             │
                    │         cos(theta_f)*sin(psi_f);              │
                    │         sin(theta_f)];                        │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           MAIN GUIDANCE LOOP START             │
                    │   WHILE (t < max_simulation_time &&            │
                    │          r <= r_prev &&                        │
                    │          norm(V_m_ECI) > min_velocity)         │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           COORDINATE TRANSFORMATION            │
                    │   R_m_local = ECI_to_Local(R_m_ECI, ...);     │
                    │   V_m_local = ECI_to_Local_vel(V_m_ECI, ...); │
                    │   x = R_m_local(1); y = R_m_local(2);         │
                    │   z = R_m_local(3); vx = V_m_local(1);        │
                    │   vy = V_m_local(2); vz = V_m_local(3);       │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           TARGET VECTOR CALCULATION            │
                    │   R_t_local = ECI_to_Local(R_t_0_ECI, ...);   │
                    │   R = [R_t_local(1); R_t_local(2);            │
                    │        R_t_local(3) + r_loop3_start] - R_m_local; │
                    │   r = norm(R);                                │
                    │   e_R = R / r;                                │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           VELOCITY CALCULATIONS                │
                    │   v = norm(V_m_local);                        │
                    │   e_m = V_m_local / v;                        │
                    │   theta = asin(vz / v);                       │
                    │   psi = atan2(vy, vx);                        │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           LOOK ANGLE CALCULATION               │
                    │   sigma = acos(dot(e_m, e_R));                │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           PHASE DECISION                       │
                    │   IF (r <= r_loop3_start)                     │
                    │     → TERMINAL PHASE                           │
                    │   ELSE                                         │
                    │     → GUIDED PHASE                             │
                    └─────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
            ┌───────────────┐            ┌───────────────────────────┐
            │ TERMINAL      │            │   GUIDED PHASE            │
            │ PHASE         │            │   CALCULATIONS            │
            │               │            │                           │
            │ A_M_local =   │            │ omega_L = cross(V_m_local, R) / r² │
            │ [0; 0; 0];    │            │ A_P = cross(N * omega_L, V_m_local); │
            │               │            │ k_L = cross(e_R, e_m) / norm(cross(e_R, e_m)); │
            │               │            │ eta = sigma / (N - 1);    │
            │               │            │ mu = sigma + eta;         │
            │               │            │ w = cos(-mu/2);           │
            │               │            │ x_q = sin(-mu/2) * k_L(1); │
            │               │            │ y_q = sin(-mu/2) * k_L(2); │
            │               │            │ z_q = sin(-mu/2) * k_L(3); │
            │               │            │ L_q = [rotation matrix];   │
            │               │            │ e_p = L_q * e_m;          │
            │               │            │ V_p = v * e_p;            │
            │               │            │ delta = acos(dot(e_f, e_p)); │
            │               │            │ l_f = cross(e_f, e_p) / norm(cross(e_f, e_p)); │
            │               │            │ t_go = r / (v * cos(sigma)); │
            │               │            │ sigma_not = sigma / sigma_max; │
            │               │            │ f_sigma_not = 1 - abs(sigma_not)^d; │
            │               │            │ delta_dot = (-K * f_sigma_not / t_go) * │
            │               │            │           (a*sig(m,delta) + b*sig(n,delta)); │
            │               │            │ omega_P = delta_dot * l_f; │
            │               │            │ A_F = cross(omega_P, V_p); │
            │               │            │ l_m = cross(k_L, e_m);     │
            │               │            │ l_P = cross(k_L, e_p);     │
            │               │            │ h_delta = a*sig(m,delta) + b*sig(n,delta); │
            │               │            │ a_p = (-N * v² * sin(sigma)) / r; │
            │               │            │ a_I_lm = ((N-1) * K * f_sigma_not * v * h_delta / t_go) * │
            │               │            │         dot(cross(l_f,e_p), l_P); │
            │               │            │ a_I_kl = (K * f_sigma_not * v * sin(sigma) * h_delta / │
            │               │            │         (t_go * sin(eta))) * dot(cross(l_f,e_p), k_L); │
            │               │            │ A_M_local = (a_p + a_I_lm) * l_m + a_I_kl * k_L; │
            └───────────────┘            └───────────────────────────┘
                    │                               │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           GUIDANCE COMMAND TO ECI              │
                    │   A_M_ECI = Local_to_ECI_vel(A_M_local, ...); │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           DRAG FORCE CALCULATION               │
                    │   v_ECI = norm(V_m_ECI);                      │
                    │   q_dyn = 0.5 * rho * v_ECI²;                │
                    │   D = q_dyn * C_d * S_ref;                    │
                    │   Dx = D * (V_m_ECI(1)) / v_ECI;             │
                    │   Dy = D * (V_m_ECI(2)) / v_ECI;             │
                    │   Dz = D * (V_m_ECI(3)) / v_ECI;             │
                    │   A_D_ECI = [Dx/m_c; Dy/m_c; Dz/m_c];        │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           GRAVITY FORCE CALCULATION            │
                    │   grav_ECI = Local_to_ECI_vel([0; 0; -g], ...); │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           TOTAL ACCELERATION                   │
                    │   A_total_ECI = A_M_ECI - A_D_ECI + grav_ECI; │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           STATE INTEGRATION                    │
                    │   V_m_ECI = V_m_ECI + A_total_ECI * dt;       │
                    │   R_m_ECI = R_m_ECI + V_m_ECI * dt;           │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           TERMINATION CHECK                    │
                    │   IF (r > r_prev)                             │
                    │     → Target impact detected                   │
                    │     → Break loop                               │
                    │   ELSE                                         │
                    │     → Continue loop                            │
                    └─────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
            ┌───────────────┐            ┌───────────────────────────┐
            │ LOOP EXIT     │            │   LOOP UPDATE             │
            │               │            │                           │
            │ Return final  │            │ r_prev = r;               │
            │ position and  │            │ t = t + dt;               │
            │ velocity      │            │ Continue to next iteration│
            └───────────────┘            └───────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           FUNCTION EXIT                       │
                    │   Return: R_m_ECI_final, V_m_ECI_final        │
                    └─────────────────────────────────────────────────┘
```

## Detailed Guided Phase Code Flow

```
                    ┌─────────────────────────────────────────────────┐
                    │           GUIDED PHASE START                   │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           ANGULAR VELOCITY CALCULATION         │
                    │   omega_L = cross(V_m_local, R) / r²;         │
                    │   A_P = cross(N * omega_L, V_m_local);        │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           GUIDANCE VECTOR CALCULATION          │
                    │   k_L = cross(e_R, e_m) / norm(cross(e_R, e_m)); │
                    │   eta = sigma / (N - 1);                      │
                    │   mu = sigma + eta;                            │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           QUATERNION CALCULATION               │
                    │   w = cos(-mu/2);                             │
                    │   x_q = sin(-mu/2) * k_L(1);                  │
                    │   y_q = sin(-mu/2) * k_L(2);                  │
                    │   z_q = sin(-mu/2) * k_L(3);                  │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           ROTATION MATRIX CONSTRUCTION         │
                    │   L_q = [w²+x_q²-y_q²-z_q², 2*(x_q*y_q-w*z_q), 2*(x_q*z_q+w*y_q); │
                    │          2*(x_q*y_q+w*z_q), w²-x_q²+y_q²-z_q², 2*(y_q*z_q-w*x_q); │
                    │          2*(x_q*z_q-w*y_q), 2*(y_q*z_q+w*x_q), w²-x_q²-y_q²+z_q²]; │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           VELOCITY PROJECTION                  │
                    │   e_p = L_q * e_m;                            │
                    │   V_p = v * e_p;                              │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           ANGLE CALCULATIONS                   │
                    │   delta = acos(dot(e_f, e_p));                │
                    │   l_f = cross(e_f, e_p) / norm(cross(e_f, e_p)); │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           TIME-TO-GO CALCULATION               │
                    │   t_go = r / (v * cos(sigma));                │
                    │   sigma_not = sigma / sigma_max;               │
                    │   f_sigma_not = 1 - abs(sigma_not)^d;         │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           GUIDANCE LAW CALCULATION             │
                    │   delta_dot = (-K * f_sigma_not / t_go) *     │
                    │              (a*sig(m,delta) + b*sig(n,delta)); │
                    │   omega_P = delta_dot * l_f;                   │
                    │   A_F = cross(omega_P, V_p);                   │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           ACCELERATION COMPONENTS              │
                    │   l_m = cross(k_L, e_m);                       │
                    │   l_P = cross(k_L, e_p);                       │
                    │   h_delta = a*sig(m,delta) + b*sig(n,delta);  │
                    │   a_p = (-N * v² * sin(sigma)) / r;           │
                    │   a_I_lm = ((N-1) * K * f_sigma_not * v * h_delta / t_go) * │
                    │           dot(cross(l_f,e_p), l_P);           │
                    │   a_I_kl = (K * f_sigma_not * v * sin(sigma) * h_delta / │
                    │           (t_go * sin(eta))) * dot(cross(l_f,e_p), k_L); │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           FINAL GUIDANCE COMMAND               │
                    │   A_M_local = (a_p + a_I_lm) * l_m + a_I_kl * k_L; │
                    └─────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────────────────┐
                    │           RETURN TO MAIN FLOW                  │
                    └─────────────────────────────────────────────────┘
```

## Data Flow Diagram

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   INPUT     │───▶│  INITIALIZE │───▶│   MAIN      │
│ PARAMETERS  │    │   PHASE     │    │   LOOP      │
│             │    │             │    │             │
│ • Launch    │    │ • Constants │    │ • State     │
│ • Target    │    │ • Target    │    │   Update    │
│ • Initial   │    │ • Initial   │    │ • Guidance  │
│ • Desired   │    │   States    │    │   Calc      │
└─────────────┘    └─────────────┘    └─────────────┘
         │                   │                   │
         │                   │                   ▼
         │                   │          ┌─────────────┐
         │                   │          │  COORDINATE │
         │                   │          │ TRANSFORM   │
         │                   │          │             │
         │                   │          │ ECI ↔ Local │
         │                   │          └─────────────┘
         │                   │                   │
         │                   │                   ▼
         │                   │          ┌─────────────┐
         │                   │          │   VECTOR    │
         │                   │          │ OPERATIONS  │
         │                   │          │             │
         │                   │          │ • Magnitude │
         │                   │          │ • Normalize │
         │                   │          │ • Cross/Dot │
         │                   │          └─────────────┘
         │                   │                   │
         │                   │                   ▼
         │                   │          ┌─────────────┐
         │                   │          │  GUIDANCE   │
         │                   │          │   LAW       │
         │                   │          │             │
         │                   │          │ • Look Angle│
         │                   │          │ • Commands  │
         │                   │          │ • Quaternion│
         │                   │          └─────────────┘
         │                   │                   │
         │                   │                   ▼
         │                   │          ┌─────────────┐
         │                   │          │   FORCE     │
         │                   │          │ CALCULATION │
         │                   │          │             │
         │                   │          │ • Drag      │
         │                   │          │ • Gravity   │
         │                   │          │ • Total     │
         │                   │          └─────────────┘
         │                   │                   │
         │                   │                   ▼
         │                   │          ┌─────────────┐
         │                   │          │  STATE      │
         │                   │          │INTEGRATION  │
         │                   │          │             │
         │                   │          │ • Velocity  │
         │                   │          │ • Position  │
         │                   │          └─────────────┘
         │                   │                   │
         │                   │                   ▼
         │                   │          ┌─────────────┐
         │                   │          │ TERMINATION │
         │                   │          │   CHECK     │
         │                   │          │             │
         │                   │          │ • Impact    │
         │                   │          │ • Time      │
         │                   │          │ • Velocity  │
         │                   │          └─────────────┘
         │                   │                   │
         │                   │                   ▼
         │                   │          ┌─────────────┐
         │                   │          │   OUTPUT    │
         │                   │          │  RESULTS    │
         │                   │          │             │
         │                   │          │ • Final     │
         │                   │          │   Position  │
         │                   │          │ • Final     │
         │                   │          │   Velocity  │
         │                   │          └─────────────┘
         │                   │
         └───────────────────┴───────────────────────────────────────┘
```

## Function Call Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│                    FUNCTION CALL SEQUENCE                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ 1. onboard_guidance_algorithm_3()                              │
│    ├── geodeticToECI()                                         │
│    ├── ECI_to_Local()                                          │
│    ├── vector_subtract()                                       │
│    ├── vector_magnitude()                                      │
│    └── WHILE LOOP START                                        │
│                                                                 │
│ 2. MAIN LOOP (Repeats until termination)                       │
│    ├── ECI_to_Local()                                          │
│    ├── ECI_to_Local_vel()                                      │
│    ├── ECI_to_Local()                                          │
│    ├── vector_subtract()                                       │
│    ├── vector_magnitude()                                      │
│    ├── vector_normalize()                                      │
│    ├── vector_magnitude()                                      │
│    ├── vector_normalize()                                      │
│    ├── vector_dot_product()                                    │
│    ├── acos()                                                  │
│    ├── asin()                                                  │
│    ├── atan2()                                                 │
│    │                                                           │
│    ├── IF (r <= r_loop3_start)                                │
│    │   └── Terminal Phase (No additional calls)               │
│    │                                                           │
│    ├── ELSE (Guided Phase)                                     │
│    │   ├── vector_cross_product()                             │
│    │   ├── vector_magnitude()                                  │
│    │   ├── vector_normalize()                                  │
│    │   ├── cos(), sin()                                        │
│    │   ├── vector_dot_product()                                │
│    │   ├── acos()                                              │
│    │   ├── vector_cross_product()                             │
│    │   ├── vector_magnitude()                                  │
│    │   ├── vector_normalize()                                  │
│    │   ├── cos()                                               │
│    │   ├── sigmoid()                                           │
│    │   ├── abs()                                               │
│    │   ├── sin()                                               │
│    │   ├── vector_cross_product()                             │
│    │   ├── vector_dot_product()                                │
│    │   └── vector_cross_product()                             │
│    │                                                           │
│    ├── Local_to_ECI_vel()                                      │
│    ├── vector_magnitude()                                      │
│    ├── vector_scale()                                          │
│    ├── Local_to_ECI_vel()                                      │
│    ├── vector_add()                                            │
│    ├── vector_subtract()                                       │
│    ├── vector_add()                                            │
│    ├── vector_add()                                            │
│    ├── rad2deg()                                               │
│    └── LOOP UPDATE                                             │
│                                                                 │
│ 3. FUNCTION EXIT                                               │
│    └── Return final states                                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Memory Flow Pattern

```
┌─────────────────────────────────────────────────────────────────┐
│                    MEMORY FLOW PATTERN                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              INPUT MEMORY                                  │ │
│ │ ┌─────────────────────────────────────────────────────────┐ │ │
│ │ │ • Launch coordinates (3 floats)                        │ │ │
│ │ │ • Target coordinates (3 floats)                        │ │ │
│ │ │ • Initial projectile states (6 floats)                │ │ │
│ │ │ • Desired impact angles (2 floats)                     │ │ │
│ │ └─────────────────────────────────────────────────────────┘ │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                │                               │
│                                ▼                               │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              CONSTANTS MEMORY                              │ │
│ │ ┌─────────────────────────────────────────────────────────┐ │ │
│ │ │ • Physical constants (6 floats)                        │ │ │
│ │ │ • Guidance parameters (7 floats)                       │ │ │
│ │ │ • Simulation parameters (4 floats)                     │ │ │
│ │ └─────────────────────────────────────────────────────────┘ │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                │                               │
│                                ▼                               │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              STATE MEMORY                                  │ │
│ │ ┌─────────────────────────────────────────────────────────┐ │ │
│ │ │ • Current ECI position (3 floats)                      │ │ │
│ │ │ • Current ECI velocity (3 floats)                      │ │ │
│ │ │ • Current local position (3 floats)                    │ │ │
│ │ │ • Current local velocity (3 floats)                    │ │ │
│ │ │ • Target ECI position (3 floats)                       │ │ │
│ │ │ • Target local position (3 floats)                     │ │ │
│ │ └─────────────────────────────────────────────────────────┘ │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                │                               │
│                                ▼                               │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              CALCULATION MEMORY                            │ │
│ │ ┌─────────────────────────────────────────────────────────┐ │ │
│ │ │ • Target vector (3 floats)                             │ │ │
│ │ │ • Unit vectors (6 floats)                              │ │ │
│ │ │ • Guidance commands (3 floats)                         │ │ │
│ │ │ • Forces (9 floats)                                    │ │ │
│ │ │ • Temporary variables (10+ floats)                     │ │ │
│ │ └─────────────────────────────────────────────────────────┘ │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                │                               │
│                                ▼                               │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              OUTPUT MEMORY                                 │ │
│ │ ┌─────────────────────────────────────────────────────────┐ │ │
│ │ │ • Final ECI position (3 floats)                        │ │ │
│ │ │ • Final ECI velocity (3 floats)                        │ │ │
│ │ │ • Impact status (1 integer)                            │ │ │
│ │ └─────────────────────────────────────────────────────────┘ │ │
│ └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```


