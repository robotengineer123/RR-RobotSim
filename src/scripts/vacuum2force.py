@staticmethod
    def vac_in_newton(center_vac_pct, edge_vac_pct):
        # type: (float, float) -> float
        """ Force calculation from generated vacuum
        Inputs Center and side vacuum in [% vacuum]
        :param center_vac_pct: vacuum in percent
        :param edge_vac_pct: vacuum in percent
        :return: vacuum in kg force
        """

        if center_vac_pct < 0:
            center_vac_pct = 0
        if edge_vac_pct < 0:
            edge_vac_pct = 0

        # Constants
        edge = 0.076674  # Combined area of side cells in foam [m2]
        center = 0.121469  # Combined area of center cells in foam [m2]
        n_to_kg = 0.1019716213  # Constant to multiply to get kg from N

        # Calculations
        # [% vacuum] converted to [Pa] and multiplied to Area [m2] to obtain [N]
        f_p_center = (center_vac_pct*1013)*center
        # [% vacuum] converted to [Pa] and multiplied to Area [m2] to obtain [N]
        f_p_side = (edge_vac_pct*1013)*edge
        # Combined force from vacuum [N]
        f_p = (f_p_center + f_p_side)*n_to_kg

        return f_p

