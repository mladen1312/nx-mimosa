#!/usr/bin/env python3
"""Generate NX-MIMOSA platform database v4.0.2 — 55+ platforms.

Hierarchical structure:
  Category → Class → Specific Platform
  
Fallback chain: Unknown → Category Generic → Class Generic → Platform

Sources: Public specifications, Jane's, IISS Military Balance, open-source intelligence.
All speeds in m/s, altitudes in m, turn rates in rad/s.
"""
import json
from collections import OrderedDict

# ===========================================================================
# Helper: standard intent phases per category
# ===========================================================================
def fighter_phases():
    return {
        "cruise":      {"prob": 0.35, "models": ["CV"],                        "q_scale": 0.3},
        "patrol":      {"prob": 0.20, "models": ["CV","CT_plus"],              "q_scale": 0.5},
        "engagement":  {"prob": 0.35, "models": ["CT_plus","CT_minus","CA"],   "q_scale": 1.5},
        "egress":      {"prob": 0.10, "models": ["CV","CA"],                   "q_scale": 0.8},
    }

def fighter_supermaneuver_phases():
    return {
        "cruise":      {"prob": 0.30, "models": ["CV"],                                "q_scale": 0.3},
        "patrol":      {"prob": 0.15, "models": ["CV","CT_plus"],                      "q_scale": 0.5},
        "engagement":  {"prob": 0.30, "models": ["CT_plus","CT_minus","CA"],           "q_scale": 1.5},
        "post_stall":  {"prob": 0.15, "models": ["Jerk","CT_plus","CT_minus"],         "q_scale": 3.0},
        "egress":      {"prob": 0.10, "models": ["CV","CA"],                           "q_scale": 0.8},
    }

def interceptor_phases():
    return {
        "cruise":     {"prob": 0.3, "models": ["CV"],            "q_scale": 0.3},
        "intercept":  {"prob": 0.5, "models": ["CV","CA"],       "q_scale": 1.0},
        "egress":     {"prob": 0.2, "models": ["CV"],            "q_scale": 0.5},
    }

def bomber_phases():
    return {
        "cruise":     {"prob": 0.6, "models": ["CV"],            "q_scale": 0.2},
        "approach":   {"prob": 0.25,"models": ["CV","CT_plus"],  "q_scale": 0.4},
        "egress":     {"prob": 0.15,"models": ["CV"],            "q_scale": 0.3},
    }

def transport_phases():
    return {
        "cruise":  {"prob": 0.7, "models": ["CV"],            "q_scale": 0.1},
        "transit": {"prob": 0.2, "models": ["CV","CT_plus"],  "q_scale": 0.3},
        "orbit":   {"prob": 0.1, "models": ["CT_plus"],       "q_scale": 0.2},
    }

def cruise_missile_phases():
    return {
        "cruise":   {"prob": 0.5, "models": ["CV"],               "q_scale": 0.3},
        "waypoint": {"prob": 0.2, "models": ["CT_plus","CT_minus"],"q_scale": 0.8},
        "terminal": {"prob": 0.3, "models": ["CA","Jerk"],        "q_scale": 2.0},
    }

def supersonic_cruise_phases():
    return {
        "cruise":   {"prob": 0.3, "models": ["CV"],                      "q_scale": 0.5},
        "sea_skim": {"prob": 0.3, "models": ["CV","CA"],                 "q_scale": 0.8},
        "terminal": {"prob": 0.4, "models": ["CA","Jerk","CT_plus"],     "q_scale": 3.0},
    }

def ballistic_phases():
    return {
        "boost":     {"prob": 0.15, "models": ["CA","Jerk"],          "q_scale": 5.0},
        "midcourse": {"prob": 0.35, "models": ["Ballistic","CV"],     "q_scale": 0.5},
        "terminal":  {"prob": 0.50, "models": ["CA","Jerk","Ballistic"],"q_scale": 8.0},
    }

def hypersonic_glide_phases():
    return {
        "boost":    {"prob": 0.1, "models": ["CA","Jerk"],              "q_scale": 5.0},
        "glide":    {"prob": 0.4, "models": ["Ballistic","CV"],         "q_scale": 1.0},
        "terminal": {"prob": 0.5, "models": ["CA","Jerk","Ballistic"],  "q_scale": 8.0},
    }

def sam_phases():
    return {
        "boost":     {"prob": 0.2, "models": ["CA","Jerk"],                  "q_scale": 4.0},
        "midcourse": {"prob": 0.3, "models": ["CV","CA"],                    "q_scale": 1.0},
        "terminal":  {"prob": 0.5, "models": ["CA","Jerk","CT_plus","CT_minus"],"q_scale": 6.0},
    }

def aam_phases():
    return {
        "boost":     {"prob": 0.15, "models": ["CA","Jerk"],                         "q_scale": 5.0},
        "cruise":    {"prob": 0.25, "models": ["CV","CA"],                            "q_scale": 1.0},
        "terminal":  {"prob": 0.60, "models": ["Jerk","CT_plus","CT_minus","CA"],     "q_scale": 8.0},
    }

def uav_phases():
    return {
        "cruise":  {"prob": 0.5, "models": ["CV"],           "q_scale": 0.1},
        "loiter":  {"prob": 0.3, "models": ["CT_plus"],      "q_scale": 0.3},
        "transit": {"prob": 0.2, "models": ["CV"],           "q_scale": 0.2},
    }

def loitering_munition_phases():
    return {
        "loiter":   {"prob": 0.4, "models": ["CV","CT_plus"],   "q_scale": 0.3},
        "transit":  {"prob": 0.3, "models": ["CV"],              "q_scale": 0.2},
        "terminal": {"prob": 0.3, "models": ["CA","CT_plus"],    "q_scale": 2.0},
    }

def helicopter_phases():
    return {
        "cruise":  {"prob": 0.3, "models": ["CV"],               "q_scale": 0.3},
        "hover":   {"prob": 0.3, "models": ["CV"],               "q_scale": 0.1},
        "nap":     {"prob": 0.25,"models": ["CT_plus","CT_minus"],"q_scale": 0.8},
        "attack":  {"prob": 0.15,"models": ["CT_plus","CA"],     "q_scale": 1.2},
    }

# ===========================================================================
# Platform definitions
# ===========================================================================
db = OrderedDict()

# --- META ---
db["_meta"] = {
    "version": "4.0.2",
    "total_platforms": 0,  # filled at end
    "categories": [
        "fighter", "interceptor", "bomber", "transport", "aew",
        "cruise_missile", "supersonic_cruise_missile", 
        "ballistic_missile", "hypersonic_missile",
        "sam", "aam", "mlrs",
        "attack_helicopter", "utility_helicopter",
        "ucav", "male_uav", "small_drone", "fpv_kamikaze", "loitering_munition",
        "unknown"
    ],
    "hierarchical_fallback": {
        "fighter":       {"parent": "manned_aircraft",  "generic_max_g": 9.0,  "generic_models": ["CV","CT_plus","CT_minus","CA"]},
        "interceptor":   {"parent": "manned_aircraft",  "generic_max_g": 5.0,  "generic_models": ["CV","CA"]},
        "bomber":        {"parent": "manned_aircraft",  "generic_max_g": 3.0,  "generic_models": ["CV"]},
        "transport":     {"parent": "manned_aircraft",  "generic_max_g": 2.5,  "generic_models": ["CV"]},
        "aew":           {"parent": "manned_aircraft",  "generic_max_g": 2.5,  "generic_models": ["CV","CT_plus"]},
        "cruise_missile":{"parent": "missile",          "generic_max_g": 5.0,  "generic_models": ["CV","CT_plus","CT_minus"]},
        "supersonic_cruise_missile": {"parent":"missile","generic_max_g":12.0, "generic_models": ["CV","CA","CT_plus"]},
        "ballistic_missile":{"parent":"missile",        "generic_max_g":25.0,  "generic_models": ["CV","CA","Ballistic"]},
        "hypersonic_missile":{"parent":"missile",       "generic_max_g":25.0,  "generic_models": ["CV","CA","Ballistic","Jerk"]},
        "sam":           {"parent": "missile",          "generic_max_g": 30.0, "generic_models": ["CV","CA","CT_plus","CT_minus","Jerk"]},
        "aam":           {"parent": "missile",          "generic_max_g": 40.0, "generic_models": ["CA","Jerk","CT_plus","CT_minus"]},
        "mlrs":          {"parent": "missile",          "generic_max_g": 5.0,  "generic_models": ["CV","Ballistic"]},
        "attack_helicopter": {"parent":"rotorcraft",    "generic_max_g": 3.5,  "generic_models": ["CV","CT_plus","CT_minus"]},
        "utility_helicopter":{"parent":"rotorcraft",    "generic_max_g": 2.5,  "generic_models": ["CV","CT_plus"]},
        "ucav":          {"parent": "uav",              "generic_max_g": 6.0,  "generic_models": ["CV","CT_plus","CT_minus","CA"]},
        "male_uav":      {"parent": "uav",             "generic_max_g": 3.0,  "generic_models": ["CV","CT_plus"]},
        "small_drone":   {"parent": "uav",             "generic_max_g": 3.0,  "generic_models": ["CV"]},
        "fpv_kamikaze":  {"parent": "uav",             "generic_max_g": 5.0,  "generic_models": ["CV","CT_plus","CA"]},
        "loitering_munition":{"parent":"uav",           "generic_max_g": 4.0,  "generic_models": ["CV","CT_plus","CA"]},
        "unknown":       {"parent": None,               "generic_max_g": 30.0, "generic_models": ["CV","CT_plus","CT_minus","CA","Jerk","Ballistic"]},
    }
}

# ===========================
# FIGHTERS (12)
# ===========================
db["F-16C"] = {
    "category": "fighter", "nato_name": "Viper",
    "max_speed_mps": 600, "cruise_speed_mps": 250, "min_speed_mps": 70,
    "max_g": 9.0, "sustained_g": 6.5, "max_turn_rate_radps": 0.35,
    "max_altitude_m": 15240,
    "typical_maneuvers": ["coordinated_turn", "high_g_break", "barrel_roll"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": 1.2
}
db["F-15E"] = {
    "category": "fighter", "nato_name": "Strike Eagle",
    "max_speed_mps": 750, "cruise_speed_mps": 270, "min_speed_mps": 75,
    "max_g": 9.0, "sustained_g": 6.0, "max_turn_rate_radps": 0.30,
    "max_altitude_m": 18300,
    "typical_maneuvers": ["coordinated_turn", "high_g_break", "immelmann"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": 5.0
}
db["F-22A"] = {
    "category": "fighter", "nato_name": "Raptor",
    "max_speed_mps": 750, "cruise_speed_mps": 500, "min_speed_mps": 60,
    "max_g": 9.0, "sustained_g": 6.5, "max_turn_rate_radps": 0.40,
    "max_altitude_m": 19800,
    "typical_maneuvers": ["coordinated_turn", "high_g_break", "post_stall", "supercruise"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA", "Jerk"],
    "initial_tpm_bias": {"CV": 0.2, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2, "Jerk": 0.1},
    "intent_phases": fighter_supermaneuver_phases(), "rcs_dbsm": -30.0
}
db["F-35A"] = {
    "category": "fighter", "nato_name": "Lightning II",
    "max_speed_mps": 520, "cruise_speed_mps": 260, "min_speed_mps": 65,
    "max_g": 9.0, "sustained_g": 5.5, "max_turn_rate_radps": 0.35,
    "max_altitude_m": 15240,
    "typical_maneuvers": ["coordinated_turn", "high_g_break"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": -30.0
}
db["F/A-18E"] = {
    "category": "fighter", "nato_name": "Super Hornet",
    "max_speed_mps": 560, "cruise_speed_mps": 250, "min_speed_mps": 65,
    "max_g": 7.5, "sustained_g": 5.5, "max_turn_rate_radps": 0.33,
    "max_altitude_m": 15240,
    "typical_maneuvers": ["coordinated_turn", "high_g_break", "carrier_approach"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": 1.0
}
db["Rafale"] = {
    "category": "fighter", "nato_name": None,
    "max_speed_mps": 600, "cruise_speed_mps": 260, "min_speed_mps": 60,
    "max_g": 9.0, "sustained_g": 6.0, "max_turn_rate_radps": 0.38,
    "max_altitude_m": 15240,
    "typical_maneuvers": ["coordinated_turn", "high_g_break", "carrier_approach"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": 0.5
}
db["Gripen_E"] = {
    "category": "fighter", "nato_name": None,
    "max_speed_mps": 680, "cruise_speed_mps": 280, "min_speed_mps": 60,
    "max_g": 9.0, "sustained_g": 6.0, "max_turn_rate_radps": 0.40,
    "max_altitude_m": 15240,
    "typical_maneuvers": ["coordinated_turn", "high_g_break"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": 0.5
}
db["Eurofighter"] = {
    "category": "fighter", "nato_name": "Typhoon",
    "max_speed_mps": 650, "cruise_speed_mps": 350, "min_speed_mps": 65,
    "max_g": 9.0, "sustained_g": 6.5, "max_turn_rate_radps": 0.42,
    "max_altitude_m": 19800,
    "typical_maneuvers": ["coordinated_turn", "high_g_break", "supercruise"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": 0.5
}
db["Su-35S"] = {
    "category": "fighter", "nato_name": "Flanker-E",
    "max_speed_mps": 700, "cruise_speed_mps": 300, "min_speed_mps": 55,
    "max_g": 9.5, "sustained_g": 7.0, "max_turn_rate_radps": 0.45,
    "max_altitude_m": 18000,
    "typical_maneuvers": ["cobra", "hook", "post_stall", "coordinated_turn"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA", "Jerk"],
    "initial_tpm_bias": {"CV": 0.2, "CT_plus": 0.2, "CT_minus": 0.2, "CA": 0.2, "Jerk": 0.2},
    "intent_phases": fighter_supermaneuver_phases(), "rcs_dbsm": 3.0
}
db["Su-57"] = {
    "category": "fighter", "nato_name": "Felon",
    "max_speed_mps": 680, "cruise_speed_mps": 480, "min_speed_mps": 55,
    "max_g": 9.5, "sustained_g": 7.0, "max_turn_rate_radps": 0.45,
    "max_altitude_m": 20000,
    "typical_maneuvers": ["cobra", "post_stall", "supercruise", "coordinated_turn"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA", "Jerk"],
    "initial_tpm_bias": {"CV": 0.2, "CT_plus": 0.2, "CT_minus": 0.2, "CA": 0.2, "Jerk": 0.2},
    "intent_phases": fighter_supermaneuver_phases(), "rcs_dbsm": -20.0
}
db["MiG-29"] = {
    "category": "fighter", "nato_name": "Fulcrum",
    "max_speed_mps": 650, "cruise_speed_mps": 280, "min_speed_mps": 65,
    "max_g": 9.0, "sustained_g": 6.0, "max_turn_rate_radps": 0.35,
    "max_altitude_m": 18000,
    "typical_maneuvers": ["coordinated_turn", "high_g_break"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": 5.0
}
db["J-20"] = {
    "category": "fighter", "nato_name": "Mighty Dragon",
    "max_speed_mps": 680, "cruise_speed_mps": 450, "min_speed_mps": 60,
    "max_g": 9.0, "sustained_g": 6.5, "max_turn_rate_radps": 0.38,
    "max_altitude_m": 20000,
    "typical_maneuvers": ["coordinated_turn", "supercruise", "high_g_break"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.25, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.25},
    "intent_phases": fighter_phases(), "rcs_dbsm": -25.0
}

# ===========================
# INTERCEPTORS (2)
# ===========================
db["MiG-31"] = {
    "category": "interceptor", "nato_name": "Foxhound",
    "max_speed_mps": 950, "cruise_speed_mps": 700, "min_speed_mps": 90,
    "max_g": 5.0, "sustained_g": 3.5, "max_turn_rate_radps": 0.20,
    "max_altitude_m": 20600,
    "typical_maneuvers": ["high_speed_intercept", "zoom_climb"],
    "preferred_models": ["CV", "CA"],
    "initial_tpm_bias": {"CV": 0.6, "CA": 0.4},
    "intent_phases": interceptor_phases(), "rcs_dbsm": 15.0
}
db["F-14D"] = {
    "category": "interceptor", "nato_name": "Tomcat",
    "max_speed_mps": 730, "cruise_speed_mps": 350, "min_speed_mps": 60,
    "max_g": 6.5, "sustained_g": 5.0, "max_turn_rate_radps": 0.28,
    "max_altitude_m": 16150,
    "typical_maneuvers": ["high_speed_intercept", "coordinated_turn"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.35, "CT_plus": 0.2, "CT_minus": 0.2, "CA": 0.25},
    "intent_phases": interceptor_phases(), "rcs_dbsm": 8.0
}

# ===========================
# BOMBERS (4)
# ===========================
db["B-52H"] = {
    "category": "bomber", "nato_name": "Stratofortress",
    "max_speed_mps": 280, "cruise_speed_mps": 230, "min_speed_mps": 80,
    "max_g": 2.5, "sustained_g": 1.5, "max_turn_rate_radps": 0.10,
    "max_altitude_m": 15150,
    "typical_maneuvers": ["level_flight", "gentle_turn"],
    "preferred_models": ["CV"],
    "initial_tpm_bias": {"CV": 1.0},
    "intent_phases": bomber_phases(), "rcs_dbsm": 40.0
}
db["B-1B"] = {
    "category": "bomber", "nato_name": "Lancer",
    "max_speed_mps": 420, "cruise_speed_mps": 290, "min_speed_mps": 85,
    "max_g": 3.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.12,
    "max_altitude_m": 18300,
    "typical_maneuvers": ["level_flight", "terrain_following", "gentle_turn"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.7, "CT_plus": 0.3},
    "intent_phases": bomber_phases(), "rcs_dbsm": 10.0
}
db["Tu-95"] = {
    "category": "bomber", "nato_name": "Bear",
    "max_speed_mps": 250, "cruise_speed_mps": 210, "min_speed_mps": 75,
    "max_g": 2.0, "sustained_g": 1.5, "max_turn_rate_radps": 0.08,
    "max_altitude_m": 12000,
    "typical_maneuvers": ["level_flight"],
    "preferred_models": ["CV"],
    "initial_tpm_bias": {"CV": 1.0},
    "intent_phases": bomber_phases(), "rcs_dbsm": 40.0
}
db["Tu-160"] = {
    "category": "bomber", "nato_name": "Blackjack",
    "max_speed_mps": 600, "cruise_speed_mps": 270, "min_speed_mps": 80,
    "max_g": 3.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.10,
    "max_altitude_m": 15600,
    "typical_maneuvers": ["level_flight", "dash", "gentle_turn"],
    "preferred_models": ["CV", "CA"],
    "initial_tpm_bias": {"CV": 0.7, "CA": 0.3},
    "intent_phases": bomber_phases(), "rcs_dbsm": 25.0
}

# ===========================
# TRANSPORT / AEW (5)
# ===========================
db["C-130"] = {
    "category": "transport", "nato_name": "Hercules",
    "max_speed_mps": 180, "cruise_speed_mps": 155, "min_speed_mps": 55,
    "max_g": 2.5, "sustained_g": 1.5, "max_turn_rate_radps": 0.08,
    "max_altitude_m": 10000,
    "typical_maneuvers": ["level_flight", "gentle_turn"],
    "preferred_models": ["CV"],
    "initial_tpm_bias": {"CV": 1.0},
    "intent_phases": transport_phases(), "rcs_dbsm": 30.0
}
db["C-17"] = {
    "category": "transport", "nato_name": "Globemaster III",
    "max_speed_mps": 260, "cruise_speed_mps": 230, "min_speed_mps": 65,
    "max_g": 2.5, "sustained_g": 1.5, "max_turn_rate_radps": 0.06,
    "max_altitude_m": 13700,
    "typical_maneuvers": ["level_flight"],
    "preferred_models": ["CV"],
    "initial_tpm_bias": {"CV": 1.0},
    "intent_phases": transport_phases(), "rcs_dbsm": 35.0
}
db["Il-76"] = {
    "category": "transport", "nato_name": "Candid",
    "max_speed_mps": 250, "cruise_speed_mps": 210, "min_speed_mps": 60,
    "max_g": 2.5, "sustained_g": 1.5, "max_turn_rate_radps": 0.06,
    "max_altitude_m": 12000,
    "typical_maneuvers": ["level_flight"],
    "preferred_models": ["CV"],
    "initial_tpm_bias": {"CV": 1.0},
    "intent_phases": transport_phases(), "rcs_dbsm": 35.0
}
db["E-2D_Hawkeye"] = {
    "category": "aew", "nato_name": "Hawkeye",
    "max_speed_mps": 180, "cruise_speed_mps": 145, "min_speed_mps": 55,
    "max_g": 2.0, "sustained_g": 1.5, "max_turn_rate_radps": 0.08,
    "max_altitude_m": 11275,
    "typical_maneuvers": ["orbit", "racetrack"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": transport_phases(), "rcs_dbsm": 25.0
}
db["E-3_AWACS"] = {
    "category": "aew", "nato_name": "Sentry",
    "max_speed_mps": 250, "cruise_speed_mps": 220, "min_speed_mps": 70,
    "max_g": 2.0, "sustained_g": 1.5, "max_turn_rate_radps": 0.06,
    "max_altitude_m": 12000,
    "typical_maneuvers": ["orbit", "racetrack"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": transport_phases(), "rcs_dbsm": 35.0
}

# ===========================
# CRUISE MISSILES (6)
# ===========================
db["Tomahawk"] = {
    "category": "cruise_missile", "nato_name": None,
    "max_speed_mps": 250, "cruise_speed_mps": 220, "min_speed_mps": 100,
    "max_g": 4.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.15,
    "max_altitude_m": 10000,
    "typical_maneuvers": ["terrain_following", "waypoint_turn", "terminal_pop_up"],
    "preferred_models": ["CV", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.5, "CT_plus": 0.25, "CT_minus": 0.25},
    "intent_phases": cruise_missile_phases(), "rcs_dbsm": -10.0
}
db["Kalibr"] = {
    "category": "cruise_missile", "nato_name": "Sizzler",
    "max_speed_mps": 300, "cruise_speed_mps": 250, "min_speed_mps": 100,
    "max_g": 5.0, "sustained_g": 3.0, "max_turn_rate_radps": 0.20,
    "max_altitude_m": 10000,
    "typical_maneuvers": ["sea_skimming", "waypoint_turn", "terminal_sprint"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.4, "CT_plus": 0.2, "CT_minus": 0.2, "CA": 0.2},
    "intent_phases": cruise_missile_phases(), "rcs_dbsm": -8.0
}
db["Storm_Shadow"] = {
    "category": "cruise_missile", "nato_name": "SCALP-EG",
    "max_speed_mps": 300, "cruise_speed_mps": 250, "min_speed_mps": 100,
    "max_g": 4.0, "sustained_g": 2.5, "max_turn_rate_radps": 0.18,
    "max_altitude_m": 10000,
    "typical_maneuvers": ["terrain_following", "terminal_dive"],
    "preferred_models": ["CV", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.4, "CT_plus": 0.3, "CT_minus": 0.3},
    "intent_phases": cruise_missile_phases(), "rcs_dbsm": -15.0
}
db["JASSM-ER"] = {
    "category": "cruise_missile", "nato_name": None,
    "max_speed_mps": 310, "cruise_speed_mps": 260, "min_speed_mps": 100,
    "max_g": 4.0, "sustained_g": 2.5, "max_turn_rate_radps": 0.15,
    "max_altitude_m": 10000,
    "typical_maneuvers": ["terrain_following", "terminal_dive"],
    "preferred_models": ["CV", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.4, "CT_plus": 0.3, "CT_minus": 0.3},
    "intent_phases": cruise_missile_phases(), "rcs_dbsm": -20.0
}
db["Harpoon"] = {
    "category": "cruise_missile", "nato_name": None,
    "max_speed_mps": 250, "cruise_speed_mps": 240, "min_speed_mps": 100,
    "max_g": 4.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.15,
    "max_altitude_m": 5000,
    "typical_maneuvers": ["sea_skimming", "pop_up", "terminal_dive"],
    "preferred_models": ["CV", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.5, "CT_plus": 0.25, "CT_minus": 0.25},
    "intent_phases": cruise_missile_phases(), "rcs_dbsm": -8.0
}
db["NSM"] = {
    "category": "cruise_missile", "nato_name": None,
    "max_speed_mps": 310, "cruise_speed_mps": 280, "min_speed_mps": 100,
    "max_g": 5.0, "sustained_g": 3.0, "max_turn_rate_radps": 0.20,
    "max_altitude_m": 5000,
    "typical_maneuvers": ["sea_skimming", "terminal_weave"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.35, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.15},
    "intent_phases": cruise_missile_phases(), "rcs_dbsm": -20.0
}

# ===========================
# SUPERSONIC CRUISE MISSILES (3)
# ===========================
db["BrahMos"] = {
    "category": "supersonic_cruise_missile", "nato_name": None,
    "max_speed_mps": 1000, "cruise_speed_mps": 850, "min_speed_mps": 200,
    "max_g": 12.0, "sustained_g": 8.0, "max_turn_rate_radps": 0.25,
    "max_altitude_m": 15000,
    "typical_maneuvers": ["sea_skimming", "terminal_dive", "s_maneuver"],
    "preferred_models": ["CV", "CA", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.3, "CA": 0.3, "CT_plus": 0.2, "CT_minus": 0.2},
    "intent_phases": supersonic_cruise_phases(), "rcs_dbsm": -5.0
}
db["P-800_Oniks"] = {
    "category": "supersonic_cruise_missile", "nato_name": "Yakhont",
    "max_speed_mps": 880, "cruise_speed_mps": 750, "min_speed_mps": 200,
    "max_g": 10.0, "sustained_g": 7.0, "max_turn_rate_radps": 0.22,
    "max_altitude_m": 14000,
    "typical_maneuvers": ["sea_skimming", "terminal_dive"],
    "preferred_models": ["CV", "CA", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.4, "CA": 0.3, "CT_plus": 0.3},
    "intent_phases": supersonic_cruise_phases(), "rcs_dbsm": -5.0
}
db["Zircon"] = {
    "category": "supersonic_cruise_missile", "nato_name": "SS-N-33",
    "max_speed_mps": 2700, "cruise_speed_mps": 2000, "min_speed_mps": 500,
    "max_g": 15.0, "sustained_g": 10.0, "max_turn_rate_radps": 0.10,
    "max_altitude_m": 40000,
    "typical_maneuvers": ["hypersonic_cruise", "terminal_dive", "terminal_weave"],
    "preferred_models": ["CV", "CA", "Jerk", "Ballistic"],
    "initial_tpm_bias": {"CV": 0.2, "CA": 0.3, "Jerk": 0.2, "Ballistic": 0.3},
    "intent_phases": supersonic_cruise_phases(), "rcs_dbsm": -10.0
}

# ===========================
# BALLISTIC MISSILES (5)
# ===========================
db["Iskander-M"] = {
    "category": "ballistic_missile", "nato_name": "SS-26 Stone",
    "max_speed_mps": 2100, "cruise_speed_mps": 1500, "min_speed_mps": 500,
    "max_g": 20.0, "sustained_g": 10.0, "max_turn_rate_radps": 0.05,
    "max_altitude_m": 50000,
    "typical_maneuvers": ["ballistic_arc", "terminal_maneuver", "evasive_weave"],
    "preferred_models": ["CV", "CA", "Ballistic"],
    "initial_tpm_bias": {"CV": 0.2, "CA": 0.3, "Ballistic": 0.5},
    "intent_phases": ballistic_phases(), "rcs_dbsm": -5.0
}
db["Tochka-U"] = {
    "category": "ballistic_missile", "nato_name": "SS-21 Scarab",
    "max_speed_mps": 1100, "cruise_speed_mps": 800, "min_speed_mps": 300,
    "max_g": 15.0, "sustained_g": 8.0, "max_turn_rate_radps": 0.03,
    "max_altitude_m": 26000,
    "typical_maneuvers": ["ballistic_arc"],
    "preferred_models": ["CV", "CA", "Ballistic"],
    "initial_tpm_bias": {"CV": 0.2, "CA": 0.2, "Ballistic": 0.6},
    "intent_phases": ballistic_phases(), "rcs_dbsm": 0.0
}
db["ATACMS"] = {
    "category": "ballistic_missile", "nato_name": None,
    "max_speed_mps": 1500, "cruise_speed_mps": 1000, "min_speed_mps": 400,
    "max_g": 15.0, "sustained_g": 8.0, "max_turn_rate_radps": 0.03,
    "max_altitude_m": 50000,
    "typical_maneuvers": ["ballistic_arc", "terminal_guidance"],
    "preferred_models": ["CV", "CA", "Ballistic"],
    "initial_tpm_bias": {"CV": 0.2, "CA": 0.2, "Ballistic": 0.6},
    "intent_phases": ballistic_phases(), "rcs_dbsm": 0.0
}
db["DF-21D"] = {
    "category": "ballistic_missile", "nato_name": "CSS-5 Mod 4",
    "max_speed_mps": 3000, "cruise_speed_mps": 2500, "min_speed_mps": 800,
    "max_g": 25.0, "sustained_g": 15.0, "max_turn_rate_radps": 0.05,
    "max_altitude_m": 300000,
    "typical_maneuvers": ["ballistic_arc", "terminal_maneuver", "maneuvering_reentry"],
    "preferred_models": ["CV", "CA", "Ballistic", "Jerk"],
    "initial_tpm_bias": {"CV": 0.15, "CA": 0.25, "Ballistic": 0.35, "Jerk": 0.25},
    "intent_phases": ballistic_phases(), "rcs_dbsm": -5.0
}
db["KN-23"] = {
    "category": "ballistic_missile", "nato_name": None,
    "max_speed_mps": 2000, "cruise_speed_mps": 1400, "min_speed_mps": 500,
    "max_g": 20.0, "sustained_g": 12.0, "max_turn_rate_radps": 0.06,
    "max_altitude_m": 50000,
    "typical_maneuvers": ["depressed_trajectory", "pull_up_maneuver", "terminal_dive"],
    "preferred_models": ["CV", "CA", "Ballistic", "Jerk"],
    "initial_tpm_bias": {"CV": 0.15, "CA": 0.3, "Ballistic": 0.3, "Jerk": 0.25},
    "intent_phases": ballistic_phases(), "rcs_dbsm": -3.0
}

# ===========================
# HYPERSONIC MISSILES (2)
# ===========================
db["Kinzhal"] = {
    "category": "hypersonic_missile", "nato_name": "Killjoy",
    "max_speed_mps": 3400, "cruise_speed_mps": 2500, "min_speed_mps": 500,
    "max_g": 25.0, "sustained_g": 12.0, "max_turn_rate_radps": 0.05,
    "max_altitude_m": 80000,
    "typical_maneuvers": ["hypersonic_glide", "terminal_dive", "pull_up"],
    "preferred_models": ["CV", "CA", "Ballistic", "Jerk"],
    "initial_tpm_bias": {"CV": 0.15, "CA": 0.25, "Ballistic": 0.35, "Jerk": 0.25},
    "intent_phases": hypersonic_glide_phases(), "rcs_dbsm": -10.0
}
db["Avangard"] = {
    "category": "hypersonic_missile", "nato_name": None,
    "max_speed_mps": 7000, "cruise_speed_mps": 6000, "min_speed_mps": 2000,
    "max_g": 20.0, "sustained_g": 10.0, "max_turn_rate_radps": 0.03,
    "max_altitude_m": 100000,
    "typical_maneuvers": ["hypersonic_glide", "skip_glide", "cross_range_maneuver"],
    "preferred_models": ["CV", "CA", "Ballistic", "Jerk"],
    "initial_tpm_bias": {"CV": 0.1, "CA": 0.2, "Ballistic": 0.4, "Jerk": 0.3},
    "intent_phases": hypersonic_glide_phases(), "rcs_dbsm": -15.0
}

# ===========================
# SAMs (5)
# ===========================
db["S-300_SAM"] = {
    "category": "sam", "nato_name": "SA-10 Grumble",
    "max_speed_mps": 2000, "cruise_speed_mps": 1200, "min_speed_mps": 400,
    "max_g": 25.0, "sustained_g": 15.0, "max_turn_rate_radps": 0.30,
    "max_altitude_m": 30000,
    "typical_maneuvers": ["proportional_nav", "terminal_lead"],
    "preferred_models": ["CV", "CA", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.2, "CA": 0.3, "CT_plus": 0.25, "CT_minus": 0.25},
    "intent_phases": sam_phases(), "rcs_dbsm": -5.0
}
db["S-400"] = {
    "category": "sam", "nato_name": "SA-21 Growler",
    "max_speed_mps": 4800, "cruise_speed_mps": 2500, "min_speed_mps": 600,
    "max_g": 25.0, "sustained_g": 15.0, "max_turn_rate_radps": 0.25,
    "max_altitude_m": 60000,
    "typical_maneuvers": ["proportional_nav", "terminal_intercept"],
    "preferred_models": ["CV", "CA", "CT_plus", "CT_minus", "Jerk"],
    "initial_tpm_bias": {"CV": 0.15, "CA": 0.25, "CT_plus": 0.2, "CT_minus": 0.2, "Jerk": 0.2},
    "intent_phases": sam_phases(), "rcs_dbsm": -8.0
}
db["Patriot_PAC3"] = {
    "category": "sam", "nato_name": None,
    "max_speed_mps": 1700, "cruise_speed_mps": 1200, "min_speed_mps": 400,
    "max_g": 30.0, "sustained_g": 20.0, "max_turn_rate_radps": 0.35,
    "max_altitude_m": 24000,
    "typical_maneuvers": ["hit_to_kill", "terminal_acceleration"],
    "preferred_models": ["CV", "CA", "Jerk", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.15, "CA": 0.25, "Jerk": 0.2, "CT_plus": 0.2, "CT_minus": 0.2},
    "intent_phases": sam_phases(), "rcs_dbsm": -5.0
}
db["THAAD"] = {
    "category": "sam", "nato_name": None,
    "max_speed_mps": 2800, "cruise_speed_mps": 2000, "min_speed_mps": 800,
    "max_g": 25.0, "sustained_g": 15.0, "max_turn_rate_radps": 0.20,
    "max_altitude_m": 150000,
    "typical_maneuvers": ["exoatmospheric_intercept", "hit_to_kill"],
    "preferred_models": ["CV", "CA", "Ballistic", "Jerk"],
    "initial_tpm_bias": {"CV": 0.15, "CA": 0.3, "Ballistic": 0.3, "Jerk": 0.25},
    "intent_phases": sam_phases(), "rcs_dbsm": -5.0
}
db["Iron_Dome_Tamir"] = {
    "category": "sam", "nato_name": None,
    "max_speed_mps": 700, "cruise_speed_mps": 500, "min_speed_mps": 100,
    "max_g": 20.0, "sustained_g": 12.0, "max_turn_rate_radps": 0.40,
    "max_altitude_m": 10000,
    "typical_maneuvers": ["proportional_nav", "terminal_lead"],
    "preferred_models": ["CV", "CA", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.2, "CA": 0.3, "CT_plus": 0.25, "CT_minus": 0.25},
    "intent_phases": sam_phases(), "rcs_dbsm": -8.0
}

# ===========================
# AIR-TO-AIR MISSILES (4)
# ===========================
db["AIM-120D"] = {
    "category": "aam", "nato_name": "AMRAAM",
    "max_speed_mps": 1400, "cruise_speed_mps": 1000, "min_speed_mps": 300,
    "max_g": 40.0, "sustained_g": 25.0, "max_turn_rate_radps": 0.50,
    "max_altitude_m": 25000,
    "typical_maneuvers": ["proportional_nav", "loft", "terminal_high_g"],
    "preferred_models": ["CA", "Jerk", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CA": 0.3, "Jerk": 0.3, "CT_plus": 0.2, "CT_minus": 0.2},
    "intent_phases": aam_phases(), "rcs_dbsm": -15.0
}
db["AIM-9X"] = {
    "category": "aam", "nato_name": "Sidewinder",
    "max_speed_mps": 850, "cruise_speed_mps": 600, "min_speed_mps": 200,
    "max_g": 50.0, "sustained_g": 35.0, "max_turn_rate_radps": 0.70,
    "max_altitude_m": 15000,
    "typical_maneuvers": ["high_off_boresight", "terminal_high_g", "lock_after_launch"],
    "preferred_models": ["Jerk", "CA", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"Jerk": 0.3, "CA": 0.3, "CT_plus": 0.2, "CT_minus": 0.2},
    "intent_phases": aam_phases(), "rcs_dbsm": -15.0
}
db["R-77"] = {
    "category": "aam", "nato_name": "AA-12 Adder",
    "max_speed_mps": 1300, "cruise_speed_mps": 900, "min_speed_mps": 300,
    "max_g": 35.0, "sustained_g": 25.0, "max_turn_rate_radps": 0.50,
    "max_altitude_m": 25000,
    "typical_maneuvers": ["proportional_nav", "terminal_high_g"],
    "preferred_models": ["CA", "Jerk", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CA": 0.3, "Jerk": 0.25, "CT_plus": 0.225, "CT_minus": 0.225},
    "intent_phases": aam_phases(), "rcs_dbsm": -12.0
}
db["Meteor"] = {
    "category": "aam", "nato_name": None,
    "max_speed_mps": 1400, "cruise_speed_mps": 1100, "min_speed_mps": 400,
    "max_g": 35.0, "sustained_g": 20.0, "max_turn_rate_radps": 0.45,
    "max_altitude_m": 25000,
    "typical_maneuvers": ["ramjet_cruise", "throttle_control", "terminal_sprint"],
    "preferred_models": ["CV", "CA", "Jerk", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.2, "CA": 0.3, "Jerk": 0.25, "CT_plus": 0.25},
    "intent_phases": aam_phases(), "rcs_dbsm": -15.0
}

# ===========================
# MLRS (2)
# ===========================
db["HIMARS_Rocket"] = {
    "category": "mlrs", "nato_name": None,
    "max_speed_mps": 900, "cruise_speed_mps": 600, "min_speed_mps": 200,
    "max_g": 3.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.02,
    "max_altitude_m": 40000,
    "typical_maneuvers": ["ballistic_arc"],
    "preferred_models": ["CV", "Ballistic"],
    "initial_tpm_bias": {"CV": 0.3, "Ballistic": 0.7},
    "intent_phases": {"ballistic": {"prob": 0.8, "models": ["Ballistic"], "q_scale": 0.5},
                       "terminal": {"prob": 0.2, "models": ["CA","Ballistic"], "q_scale": 2.0}},
    "rcs_dbsm": 0.0
}
db["BM-21_Grad"] = {
    "category": "mlrs", "nato_name": None,
    "max_speed_mps": 700, "cruise_speed_mps": 500, "min_speed_mps": 200,
    "max_g": 2.0, "sustained_g": 1.5, "max_turn_rate_radps": 0.01,
    "max_altitude_m": 15000,
    "typical_maneuvers": ["ballistic_arc"],
    "preferred_models": ["CV", "Ballistic"],
    "initial_tpm_bias": {"CV": 0.2, "Ballistic": 0.8},
    "intent_phases": {"ballistic": {"prob": 1.0, "models": ["Ballistic"], "q_scale": 0.3}},
    "rcs_dbsm": 0.0
}

# ===========================
# HELICOPTERS (4)
# ===========================
db["AH-64_Apache"] = {
    "category": "attack_helicopter", "nato_name": None,
    "max_speed_mps": 90, "cruise_speed_mps": 70, "min_speed_mps": 0,
    "max_g": 3.5, "sustained_g": 2.5, "max_turn_rate_radps": 0.30,
    "max_altitude_m": 6400,
    "typical_maneuvers": ["nap_of_earth", "bob_up", "sharp_break_turn", "hover"],
    "preferred_models": ["CV", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.4, "CT_plus": 0.3, "CT_minus": 0.3},
    "intent_phases": helicopter_phases(), "rcs_dbsm": 10.0
}
db["Ka-52"] = {
    "category": "attack_helicopter", "nato_name": "Hokum-B",
    "max_speed_mps": 85, "cruise_speed_mps": 65, "min_speed_mps": 0,
    "max_g": 3.5, "sustained_g": 2.5, "max_turn_rate_radps": 0.35,
    "max_altitude_m": 5500,
    "typical_maneuvers": ["nap_of_earth", "barrel_roll_heli", "sharp_break_turn"],
    "preferred_models": ["CV", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.35, "CT_minus": 0.35},
    "intent_phases": helicopter_phases(), "rcs_dbsm": 10.0
}
db["Mi-28"] = {
    "category": "attack_helicopter", "nato_name": "Havoc",
    "max_speed_mps": 85, "cruise_speed_mps": 70, "min_speed_mps": 0,
    "max_g": 3.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.25,
    "max_altitude_m": 5600,
    "typical_maneuvers": ["nap_of_earth", "hover", "attack_run"],
    "preferred_models": ["CV", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.4, "CT_plus": 0.3, "CT_minus": 0.3},
    "intent_phases": helicopter_phases(), "rcs_dbsm": 10.0
}
db["UH-60_Black_Hawk"] = {
    "category": "utility_helicopter", "nato_name": None,
    "max_speed_mps": 80, "cruise_speed_mps": 65, "min_speed_mps": 0,
    "max_g": 2.5, "sustained_g": 1.5, "max_turn_rate_radps": 0.20,
    "max_altitude_m": 5800,
    "typical_maneuvers": ["cruise", "hover", "approach"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": helicopter_phases(), "rcs_dbsm": 12.0
}

# ===========================
# UCAV (2)
# ===========================
db["XQ-58_Valkyrie"] = {
    "category": "ucav", "nato_name": None,
    "max_speed_mps": 310, "cruise_speed_mps": 250, "min_speed_mps": 60,
    "max_g": 6.0, "sustained_g": 4.0, "max_turn_rate_radps": 0.25,
    "max_altitude_m": 13700,
    "typical_maneuvers": ["coordinated_turn", "dive", "autonomous_engagement"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": -15.0
}
db["S-70_Okhotnik"] = {
    "category": "ucav", "nato_name": "Hunter-B",
    "max_speed_mps": 280, "cruise_speed_mps": 230, "min_speed_mps": 55,
    "max_g": 5.0, "sustained_g": 3.0, "max_turn_rate_radps": 0.20,
    "max_altitude_m": 18000,
    "typical_maneuvers": ["cruise", "coordinated_turn", "dive"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.35, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.15},
    "intent_phases": fighter_phases(), "rcs_dbsm": -20.0
}

# ===========================
# MALE UAVs (4)
# ===========================
db["MQ-9_Reaper"] = {
    "category": "male_uav", "nato_name": None,
    "max_speed_mps": 130, "cruise_speed_mps": 80, "min_speed_mps": 40,
    "max_g": 3.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.15,
    "max_altitude_m": 15000,
    "typical_maneuvers": ["loiter", "orbit", "cruise"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": uav_phases(), "rcs_dbsm": 1.0
}
db["TB2_Bayraktar"] = {
    "category": "male_uav", "nato_name": None,
    "max_speed_mps": 70, "cruise_speed_mps": 40, "min_speed_mps": 20,
    "max_g": 2.5, "sustained_g": 1.5, "max_turn_rate_radps": 0.12,
    "max_altitude_m": 8200,
    "typical_maneuvers": ["loiter", "orbit", "attack_dive"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": uav_phases(), "rcs_dbsm": 0.5
}
db["Wing_Loong_II"] = {
    "category": "male_uav", "nato_name": None,
    "max_speed_mps": 95, "cruise_speed_mps": 55, "min_speed_mps": 30,
    "max_g": 3.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.12,
    "max_altitude_m": 9000,
    "typical_maneuvers": ["loiter", "orbit"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": uav_phases(), "rcs_dbsm": 0.5
}
db["Mohajer-6"] = {
    "category": "male_uav", "nato_name": None,
    "max_speed_mps": 55, "cruise_speed_mps": 35, "min_speed_mps": 20,
    "max_g": 2.5, "sustained_g": 1.5, "max_turn_rate_radps": 0.10,
    "max_altitude_m": 5500,
    "typical_maneuvers": ["loiter", "orbit"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": uav_phases(), "rcs_dbsm": 0.0
}

# ===========================
# SMALL DRONES / FPV / LOITERING (5)
# ===========================
db["DJI_Mavic"] = {
    "category": "small_drone", "nato_name": None,
    "max_speed_mps": 20, "cruise_speed_mps": 10, "min_speed_mps": 0,
    "max_g": 2.0, "sustained_g": 1.0, "max_turn_rate_radps": 0.50,
    "max_altitude_m": 500,
    "typical_maneuvers": ["hover", "slow_orbit", "photo_pass"],
    "preferred_models": ["CV"],
    "initial_tpm_bias": {"CV": 1.0},
    "intent_phases": {"hover": {"prob": 0.4, "models": ["CV"], "q_scale": 0.05},
                       "transit": {"prob": 0.6, "models": ["CV"], "q_scale": 0.1}},
    "rcs_dbsm": -20.0
}
db["FPV_Drone"] = {
    "category": "fpv_kamikaze", "nato_name": None,
    "max_speed_mps": 40, "cruise_speed_mps": 25, "min_speed_mps": 5,
    "max_g": 4.0, "sustained_g": 2.5, "max_turn_rate_radps": 1.50,
    "max_altitude_m": 500,
    "typical_maneuvers": ["aggressive_turn", "dive_attack", "nap_of_earth"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": loitering_munition_phases(), "rcs_dbsm": -25.0
}
db["Shahed_136"] = {
    "category": "loitering_munition", "nato_name": "Geran-2",
    "max_speed_mps": 55, "cruise_speed_mps": 46, "min_speed_mps": 30,
    "max_g": 3.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.15,
    "max_altitude_m": 4000,
    "typical_maneuvers": ["loiter", "cruise", "terminal_dive"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": loitering_munition_phases(), "rcs_dbsm": -5.0
}
db["Lancet"] = {
    "category": "loitering_munition", "nato_name": None,
    "max_speed_mps": 80, "cruise_speed_mps": 30, "min_speed_mps": 15,
    "max_g": 4.0, "sustained_g": 3.0, "max_turn_rate_radps": 0.30,
    "max_altitude_m": 5000,
    "typical_maneuvers": ["loiter", "terminal_dive", "target_acquisition"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": loitering_munition_phases(), "rcs_dbsm": -15.0
}
db["Switchblade_600"] = {
    "category": "loitering_munition", "nato_name": None,
    "max_speed_mps": 50, "cruise_speed_mps": 30, "min_speed_mps": 15,
    "max_g": 3.5, "sustained_g": 2.0, "max_turn_rate_radps": 0.25,
    "max_altitude_m": 3000,
    "typical_maneuvers": ["loiter", "terminal_dive"],
    "preferred_models": ["CV", "CT_plus", "CA"],
    "initial_tpm_bias": {"CV": 0.4, "CT_plus": 0.3, "CA": 0.3},
    "intent_phases": loitering_munition_phases(), "rcs_dbsm": -15.0
}

# ===========================
# FALLBACK / UNKNOWN GENERICS (5)
# ===========================
db["Unknown"] = {
    "category": "unknown", "nato_name": None,
    "max_speed_mps": 3500, "cruise_speed_mps": 500, "min_speed_mps": 0,
    "max_g": 30.0, "sustained_g": 15.0, "max_turn_rate_radps": 1.5,
    "max_altitude_m": 100000,
    "typical_maneuvers": ["any"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA", "Jerk", "Ballistic"],
    "initial_tpm_bias": {"CV": 0.25, "CT_plus": 0.15, "CT_minus": 0.15, "CA": 0.2, "Jerk": 0.1, "Ballistic": 0.15},
    "intent_phases": {"unknown": {"prob": 1.0, "models": ["CV","CT_plus","CT_minus","CA","Jerk","Ballistic"], "q_scale": 1.0}},
    "rcs_dbsm": 0.0
}
db["Generic_Fighter"] = {
    "category": "fighter", "nato_name": None,
    "max_speed_mps": 650, "cruise_speed_mps": 280, "min_speed_mps": 60,
    "max_g": 9.0, "sustained_g": 6.0, "max_turn_rate_radps": 0.38,
    "max_altitude_m": 18000,
    "typical_maneuvers": ["coordinated_turn", "high_g_break"],
    "preferred_models": ["CV", "CT_plus", "CT_minus", "CA"],
    "initial_tpm_bias": {"CV": 0.3, "CT_plus": 0.25, "CT_minus": 0.25, "CA": 0.2},
    "intent_phases": fighter_phases(), "rcs_dbsm": 3.0
}
db["Generic_Cruise_Missile"] = {
    "category": "cruise_missile", "nato_name": None,
    "max_speed_mps": 300, "cruise_speed_mps": 250, "min_speed_mps": 100,
    "max_g": 5.0, "sustained_g": 3.0, "max_turn_rate_radps": 0.18,
    "max_altitude_m": 10000,
    "typical_maneuvers": ["terrain_following", "waypoint_turn", "terminal_dive"],
    "preferred_models": ["CV", "CT_plus", "CT_minus"],
    "initial_tpm_bias": {"CV": 0.5, "CT_plus": 0.25, "CT_minus": 0.25},
    "intent_phases": cruise_missile_phases(), "rcs_dbsm": -10.0
}
db["Generic_Ballistic"] = {
    "category": "ballistic_missile", "nato_name": None,
    "max_speed_mps": 2500, "cruise_speed_mps": 1500, "min_speed_mps": 500,
    "max_g": 20.0, "sustained_g": 10.0, "max_turn_rate_radps": 0.05,
    "max_altitude_m": 100000,
    "typical_maneuvers": ["ballistic_arc", "terminal_maneuver"],
    "preferred_models": ["CV", "CA", "Ballistic"],
    "initial_tpm_bias": {"CV": 0.2, "CA": 0.3, "Ballistic": 0.5},
    "intent_phases": ballistic_phases(), "rcs_dbsm": 0.0
}
db["Generic_UAV"] = {
    "category": "male_uav", "nato_name": None,
    "max_speed_mps": 80, "cruise_speed_mps": 40, "min_speed_mps": 10,
    "max_g": 3.0, "sustained_g": 2.0, "max_turn_rate_radps": 0.15,
    "max_altitude_m": 8000,
    "typical_maneuvers": ["loiter", "orbit", "cruise"],
    "preferred_models": ["CV", "CT_plus"],
    "initial_tpm_bias": {"CV": 0.6, "CT_plus": 0.4},
    "intent_phases": uav_phases(), "rcs_dbsm": 0.0
}

# Count and save
total = len([k for k in db if k not in ('_meta',)])
db["_meta"]["total_platforms"] = total
print(f"Generated {total} platforms")

with open('data/platform_db.json', 'w') as f:
    json.dump(db, f, indent=2)
print("Saved to data/platform_db.json")
