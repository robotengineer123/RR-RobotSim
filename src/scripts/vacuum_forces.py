from vacuum2force import vac_in_newton

#%% Constansts
mm_to_m = 0.001

# From foam data sheet
p_25 = 3000     # Pressure to compress foam material 25 % [Pa]
p_50 = 6500     # Pressure to compress foam material 50 % [Pa]

# From file vacuum2force.py
edge = 0.076674  # Combined area of side cells in foam [m2]
center = 0.121469  # Combined area of center cells in foam [m2]

# From drawings,
A_tot = 520*mm_to_m*700*mm_to_m     # Total area [m2]
A_cells = edge + center             # Area of vacuum cells [m2]
A_foam = A_tot - A_cells            # Area of foam [m2]
t = (55.83+65.82)*mm_to_m/2         # Thickness of vacuum sheet [m] average from CAD
w_to_base = 49.55*mm_to_m           # Wheel to base og vacuum sheet
d_v = t - w_to_base                 # Distance traveled before contact between wheels and blade

# Turbine blade: Polyurethan (PU) paint (https://www.lowtechmagazine.com/2019/06/wooden-wind-turbines.html)
# Vacuum film: Polyethylene film
# Assume polyethylene-polyethylene, clean and dry (https://www.engineeringtoolbox.com/friction-coefficients-d_778.html)
mu = 0.2

# Stiffness of vacuum sheet
d_comp_25 = t*0.25      # Compression distance of 25 % [m]
F_25 = p_25*A_foam      # Force to compress sheet 25 % [N]
k_25 = F_25/d_comp_25   # Stiffness of vacuum sheet [N/m]

d_comp_50 = t*0.50      # Compression distance of 25 % [m]
F_50 = p_50*A_foam      # Force to compress sheet 25 % [N]
k_50 = F_50/d_comp_50   # Stiffness of vacuum sheet [N/m]

# Force to compress vacuum sheet until wheel contact (using 25 % compression)
N_v = k_25*d_v

#%% Variables
# Is vacuum on or off
center_vac = True
edge_vac = True

# Vacuum percentage
vac_pct = 10

# Evaluate vacuum pct
center_vac_pct = 0
edge_vac_pct = 0
if center_vac:
    center_vac_pct = vac_pct
if edge_vac:
    edge_vac_pct = vac_pct

# Force from vacuum(center vacuum %, edge vacuum percent)
N = vac_in_newton(center_vac_pct, edge_vac_pct)

# Force to compress suspension
N_s = N - N_v

# Friction force from vacuum sheet
F = mu*N_v

print(f'The normal force is N={N} N and the friction force is F={F} N')