from vacuum2force import vac_in_newton

def vac_stiffness():
    """
    Stiffness of the vacuum sheet
    """

    # Constansts
    mm_to_m = 0.001

    # From data sheet
    p_25 = 3000     # Pressure to compress foam material 25 % [Pa]
    p_50 = 6500     # Pressure to compress foam material 50 % [Pa]

    edge = 0.076674  # Combined area of side cells in foam [m2]
    center = 0.121469  # Combined area of center cells in foam [m2]

    # From drawings,
    A_tot = 520*mm_to_m*700*mm_to_m     # Total area [m2]
    A_cells = edge + center             # Area of vacuum cells [m2]
    A_foam = A_tot - A_cells            # Area of foam [m2]
    t = 35*mm_to_m                      # Thickness of vacuum sheet [m]

    # Stiffness of vacuum sheet
    d_comp_25 = t*0.25      # Compression distance of 25 % [m]
    F_25 = p_25*A_foam      # Force to compress sheet 25 % [N]
    k_25 = F_25/d_comp_25      # Stiffness of vacuum sheet [N/m]

    d_comp_50 = t*0.50      # Compression distance of 25 % [m]
    F_50 = p_50*A_foam      # Force to compress sheet 25 % [N]
    k_50 = F_50/d_comp_50      # Stiffness of vacuum sheet [N/m]

    return k_25, k_50


#%%
k_25, k_50 = vac_stiffness()
F = vac_in_newton(20, 20)
