import math

# Pour chaque formation : dict robot_index -> (range_m, bearing_deg)
# bearing > 0 = leader vu a gauche -> le follower se place a droite.
FORMATIONS = {
    "colonne": {
        2: (0.6, 0.0),
        3: (1.2, 0.0),
        4: (1.8, 0.0),
    },
    "ligne": {
        2: (0.6,  30.0),
        3: (0.6, -30.0),
        4: (1.0,   0.0),
    },
    "triangle": {
        2: (0.6,  25.0),
        3: (0.6, -25.0),
        4: (1.2,   0.0),
    },
    "carre": {
        2: (0.6,  20.0),
        3: (0.6, -20.0),
        4: (1.0,   0.0),
    },
}


def get_offset(formation, robot_index):
    """Retourne (range_m, bearing_rad). Fallback colonne si absent."""
    table = FORMATIONS.get(formation, FORMATIONS["colonne"])
    rng, bearing_deg = table.get(robot_index, (0.6 * (robot_index - 1), 0.0))
    return rng, math.radians(bearing_deg)
