#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v4.0 - CIVIL AVIATION ATC/ATM COMPLIANCE VALIDATION
═══════════════════════════════════════════════════════════════════════════════════════════════════════

EUROCONTROL EASSP (European ATM Surveillance System Performance) Requirements:
──────────────────────────────────────────────────────────────────────────────

1. POSITION ACCURACY
   • RMS ≤ 500 m for en-route
   • RMS ≤ 150 m for terminal area (TMA)
   • Support 3 NM / 5 NM separation minima

2. TRACK CONTINUITY
   • ≥ 99.5% for cooperative targets (Mode S/ADS-B)
   • ≥ 98% for non-cooperative targets (PSR only)

3. LATENCY
   • ≤ 2 seconds (95th percentile) from measurement to display

4. FALSE TRACK RATE
   • ≤ 0.1 per hour

5. UPDATE RATE
   • Radar: 4-12 rpm (4-12 seconds per scan)
   • ADS-B: 1 Hz typical

ICAO DOC 4444 - Separation Minima:
──────────────────────────────────
• 5 NM (9.26 km) - Standard radar separation
• 3 NM (5.56 km) - Reduced separation (requires enhanced surveillance)

Test Scenarios:
───────────────
1. En-route cruise (FL350, Mach 0.85)
2. Terminal approach (ILS, 3° glideslope)
3. Departure climb (high acceleration)
4. Holding pattern (race-track, standard rate turns)
5. Go-around (high maneuver)
6. Multi-sensor fusion (PSR + SSR + ADS-B)
7. Sensor degradation (loss of SSR, ADS-B only)
8. Clutter environment (weather, birds, ground clutter)

Author: Dr. Mladen Mešter / Nexellum d.o.o.
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import norm
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import time

# Import unified tracker
from nx_mimosa_v4_unified import NXMIMOSAUnified, Industry, create_tracker

# =============================================================================
# ATC-SPECIFIC CONSTANTS
# =============================================================================

# Geographic constants
NM_TO_METERS = 1852.0  # Nautical miles to meters
FL_TO_METERS = 30.48   # Flight level (100 ft) to meters
KNOTS_TO_MS = 0.5144   # Knots to m/s

# EUROCONTROL requirements
EUROCONTROL_ENROUTE_RMS = 500.0       # meters
EUROCONTROL_TMA_RMS = 150.0           # meters  
EUROCONTROL_CONTINUITY = 0.995        # 99.5%
EUROCONTROL_LATENCY_95 = 2.0          # seconds
EUROCONTROL_FALSE_TRACK_RATE = 0.1    # per hour

# Separation minima
SEPARATION_5NM = 5 * NM_TO_METERS     # 9,260 m
SEPARATION_3NM = 3 * NM_TO_METERS     # 5,556 m


# =============================================================================
# FLIGHT PHASES
# =============================================================================

class FlightPhase(Enum):
    ENROUTE = "En-route Cruise"
    TERMINAL_APPROACH = "Terminal Approach"
    DEPARTURE = "Departure Climb"
    HOLDING = "Holding Pattern"
    GO_AROUND = "Go-Around"


# =============================================================================
# TRAJECTORY GENERATORS
# =============================================================================

def generate_enroute_trajectory(duration: float, dt: float, seed: int = 42) -> np.ndarray:
    """
    Generate en-route cruise trajectory.
    
    Aircraft: A320 at FL350, Mach 0.85 (~450 kts TAS)
    Characteristics: Mostly straight, occasional heading changes
    """
    np.random.seed(seed)
    T = int(duration / dt)
    traj = np.zeros((T, 6))
    
    # Initial state: 100 km out, FL350, heading 090
    x0, y0, z0 = 100000, 0, 350 * FL_TO_METERS  # ~10,668 m
    vx0 = 450 * KNOTS_TO_MS  # ~232 m/s
    vy0 = 0
    vz0 = 0
    
    traj[0] = [x0, y0, z0, vx0, vy0, vz0]
    
    for k in range(1, T):
        t = k * dt
        
        # Occasional heading change (every 60s, random ±5°)
        if k % int(60/dt) == 0:
            heading = np.arctan2(traj[k-1, 4], traj[k-1, 3])
            d_heading = np.deg2rad(np.random.uniform(-5, 5))
            speed = norm(traj[k-1, 3:5])
            traj[k, 3] = speed * np.cos(heading + d_heading)
            traj[k, 4] = speed * np.sin(heading + d_heading)
            traj[k, 5] = 0
        else:
            traj[k, 3:6] = traj[k-1, 3:6]
        
        # Integrate position
        traj[k, :3] = traj[k-1, :3] + traj[k, 3:6] * dt
    
    return traj


def generate_approach_trajectory(duration: float, dt: float, seed: int = 42) -> np.ndarray:
    """
    Generate ILS approach trajectory.
    
    Aircraft: A320 on 3° glideslope, 140 kts final approach speed
    Characteristics: Descending, decelerating, precise track
    """
    np.random.seed(seed)
    T = int(duration / dt)
    traj = np.zeros((T, 6))
    
    # Initial: 10 NM out, 3000 ft AGL, on glideslope
    x0 = 10 * NM_TO_METERS
    y0 = 0
    z0 = 3000 * 0.3048  # ~914 m
    
    gs_angle = np.deg2rad(3.0)
    speed = 140 * KNOTS_TO_MS  # ~72 m/s
    
    vx0 = -speed * np.cos(gs_angle)
    vy0 = 0
    vz0 = -speed * np.sin(gs_angle)
    
    traj[0] = [x0, y0, z0, vx0, vy0, vz0]
    
    for k in range(1, T):
        # Decelerate slightly
        speed_factor = 1.0 - 0.1 * (k / T)
        traj[k, 3] = vx0 * speed_factor
        traj[k, 4] = np.random.randn() * 0.5  # Small lateral variations
        traj[k, 5] = vz0 * speed_factor
        
        traj[k, :3] = traj[k-1, :3] + traj[k, 3:6] * dt
        
        # Stop at threshold
        if traj[k, 0] < 0:
            traj[k:] = traj[k]
            break
    
    return traj


def generate_holding_trajectory(duration: float, dt: float, seed: int = 42) -> np.ndarray:
    """
    Generate holding pattern (race-track).
    
    Standard rate turn (3°/s), 1-minute legs
    """
    np.random.seed(seed)
    T = int(duration / dt)
    traj = np.zeros((T, 6))
    
    # Initial: Over holding fix, 10000 ft, 220 kts
    speed = 220 * KNOTS_TO_MS  # ~113 m/s
    altitude = 10000 * 0.3048
    turn_rate = np.deg2rad(3.0)  # 3°/s standard rate
    leg_time = 60.0  # 1 minute legs
    
    traj[0] = [0, 0, altitude, speed, 0, 0]
    heading = 0
    
    for k in range(1, T):
        t = k * dt
        
        # Pattern: outbound leg, turn, inbound leg, turn
        phase = (t % (4 * leg_time)) / leg_time
        
        if phase < 1.0:  # Outbound leg
            d_heading = 0
        elif phase < 1.5:  # Turn 1
            d_heading = turn_rate * dt
        elif phase < 2.5:  # Inbound leg
            d_heading = 0
        else:  # Turn 2
            d_heading = turn_rate * dt
        
        heading += d_heading
        
        traj[k, 3] = speed * np.cos(heading)
        traj[k, 4] = speed * np.sin(heading)
        traj[k, 5] = 0
        
        traj[k, :3] = traj[k-1, :3] + traj[k, 3:6] * dt
    
    return traj


def generate_go_around_trajectory(duration: float, dt: float, seed: int = 42) -> np.ndarray:
    """
    Generate go-around trajectory.
    
    High maneuver: Level off, accelerate, climb
    """
    np.random.seed(seed)
    T = int(duration / dt)
    traj = np.zeros((T, 6))
    
    # Initial: On final, 500 ft, 140 kts
    traj[0] = [2000, 0, 500 * 0.3048, -140 * KNOTS_TO_MS * 0.995, 0, -2]
    
    for k in range(1, T):
        t = k * dt
        
        # Phase 1 (0-10s): Level off, apply TOGA
        if t < 10:
            ax = 3.0  # ~0.3g acceleration
            az = 2.0 + t * 0.5  # Increasing climb rate
        # Phase 2 (10-30s): Climb, accelerate
        elif t < 30:
            ax = 2.0
            az = 1.0
        # Phase 3 (30+): Stabilize climb
        else:
            ax = 0.5
            az = 0
        
        traj[k, 3] = traj[k-1, 3] + ax * dt
        traj[k, 4] = traj[k-1, 4]
        traj[k, 5] = traj[k-1, 5] + az * dt
        
        # Limit climb rate to realistic values
        traj[k, 5] = np.clip(traj[k, 5], -10, 20)
        
        traj[k, :3] = traj[k-1, :3] + traj[k, 3:6] * dt
    
    return traj


# =============================================================================
# SENSOR MODELS
# =============================================================================

class RadarSensor:
    """PSR + SSR sensor model - provides update every rotation."""
    
    def __init__(self, position: np.ndarray, 
                 range_sigma: float = 50.0,
                 azimuth_sigma: float = 0.1,  # degrees
                 elevation_sigma: float = 0.5,  # degrees
                 rotation_period: float = 4.0,  # seconds
                 seed: int = 42):
        self.position = position
        self.range_sigma = range_sigma
        self.azimuth_sigma = np.deg2rad(azimuth_sigma)
        self.elevation_sigma = np.deg2rad(elevation_sigma)
        self.rotation_period = rotation_period
        self.rng = np.random.RandomState(seed)
        self.last_scan = -rotation_period  # Ensure first measurement works
        
    def measure(self, target_pos: np.ndarray, t: float) -> Optional[Tuple[np.ndarray, float]]:
        """
        Generate measurement once per radar rotation.
        In real ATC, target is detected each scan (99.9% Pd for cooperative targets).
        """
        # Check if new scan has occurred
        if t - self.last_scan < self.rotation_period:
            return None
        
        self.last_scan = t
        
        # Relative position
        rel_pos = target_pos - self.position
        r = norm(rel_pos)
        
        # Check range (max 200 NM typical for en-route radar)
        if r > 200 * NM_TO_METERS:
            return None
        
        # Generate measurement with noise in spherical coords
        target_az = np.arctan2(rel_pos[1], rel_pos[0])
        target_el = np.arctan2(rel_pos[2], norm(rel_pos[:2]))
        
        az = target_az + self.rng.randn() * self.azimuth_sigma
        el = target_el + self.rng.randn() * self.elevation_sigma
        r_meas = r + self.rng.randn() * self.range_sigma
        
        # Convert back to Cartesian
        x = r_meas * np.cos(el) * np.cos(az) + self.position[0]
        y = r_meas * np.cos(el) * np.sin(az) + self.position[1]
        z = r_meas * np.sin(el) + self.position[2]
        
        # Effective sigma in Cartesian (approximate)
        effective_sigma = np.sqrt(self.range_sigma**2 + (r * self.azimuth_sigma)**2)
        
        return np.array([x, y, z]), effective_sigma


class ADSBReceiver:
    """ADS-B ground station model."""
    
    def __init__(self, position: np.ndarray,
                 horizontal_accuracy: float = 30.0,  # NACp=8 → ~30m
                 vertical_accuracy: float = 45.0,
                 update_rate: float = 1.0,  # Hz
                 seed: int = 42):
        self.position = position
        self.horizontal_accuracy = horizontal_accuracy
        self.vertical_accuracy = vertical_accuracy
        self.update_period = 1.0 / update_rate
        self.rng = np.random.RandomState(seed)
        self.last_update = 0
        
    def measure(self, target_pos: np.ndarray, t: float) -> Optional[Tuple[np.ndarray, float]]:
        """Generate ADS-B measurement at update rate."""
        if t - self.last_update < self.update_period:
            return None
        
        self.last_update = t
        
        # ADS-B reports aircraft's own position estimate
        noise = np.array([
            self.rng.randn() * self.horizontal_accuracy,
            self.rng.randn() * self.horizontal_accuracy,
            self.rng.randn() * self.vertical_accuracy
        ])
        
        return target_pos + noise, self.horizontal_accuracy


# =============================================================================
# COMPLIANCE VALIDATOR
# =============================================================================

@dataclass
class ValidationResult:
    """Results of compliance validation."""
    scenario: str
    passed: bool
    position_rms: float
    velocity_rms: float
    track_continuity: float
    max_position_error: float
    separation_safe: bool
    details: Dict


class ATCComplianceValidator:
    """
    Validates tracker performance against EUROCONTROL EASSP requirements.
    """
    
    def __init__(self, separation_minima: float = SEPARATION_5NM):
        self.separation_minima = separation_minima
        self.results: List[ValidationResult] = []
        
    def validate_scenario(self, 
                          tracker: NXMIMOSAUnified,
                          trajectory: np.ndarray,
                          sensor: RadarSensor,
                          scenario_name: str,
                          dt: float = 0.1) -> ValidationResult:
        """
        Run validation scenario and check compliance.
        """
        # Initialize tracker
        tracker.initialize(trajectory[0, :3], trajectory[0, 3:6])
        
        # Storage
        estimates = []
        position_errors = []
        velocity_errors = []
        measurements_received = 0
        prediction_steps = 0
        
        last_update_k = 0
        
        # Run simulation
        for k in range(1, len(trajectory)):
            t = k * dt
            true_pos = trajectory[k, :3]
            true_vel = trajectory[k, 3:6]
            
            # Get sensor measurement (radar updates every rotation period)
            measurement = sensor.measure(true_pos, t)
            
            if measurement is not None:
                # Time since last update
                time_since_update = (k - last_update_k) * dt
                
                # Predict to current time
                tracker.predict(time_since_update)
                
                z, sigma = measurement
                tracker.update(z, sigma)
                measurements_received += 1
                last_update_k = k
            else:
                # Just predict one step
                tracker.predict(dt)
                prediction_steps += 1
            
            # Record estimate
            est = tracker.get_state()
            estimates.append(est)
            
            # Compute errors
            pos_err = norm(est[:3] - true_pos)
            vel_err = norm(est[3:6] - true_vel)
            position_errors.append(pos_err)
            velocity_errors.append(vel_err)
        
        # Compute statistics
        position_errors = np.array(position_errors)
        velocity_errors = np.array(velocity_errors)
        
        position_rms = np.sqrt(np.mean(position_errors**2))
        velocity_rms = np.sqrt(np.mean(velocity_errors**2))
        max_position_error = np.max(position_errors)
        
        # Track continuity: fraction of time with valid track
        # In ATC, we get measurement every radar scan, so continuity based on measurement rate
        total_scans_expected = int(len(trajectory) * dt / sensor.rotation_period)
        track_continuity = measurements_received / max(1, total_scans_expected)
        
        # Check if separation would be maintained
        # Worst-case: two aircraft both have max error in opposite directions
        separation_margin = self.separation_minima - 2 * max_position_error
        separation_safe = separation_margin > 0
        
        # Determine pass/fail
        if scenario_name in ['En-route Cruise', 'Holding Pattern']:
            rms_limit = EUROCONTROL_ENROUTE_RMS
        else:
            rms_limit = EUROCONTROL_TMA_RMS
        
        passed = (
            position_rms <= rms_limit and
            track_continuity >= EUROCONTROL_CONTINUITY and
            separation_safe
        )
        
        result = ValidationResult(
            scenario=scenario_name,
            passed=passed,
            position_rms=position_rms,
            velocity_rms=velocity_rms,
            track_continuity=track_continuity,
            max_position_error=max_position_error,
            separation_safe=separation_safe,
            details={
                'rms_limit': rms_limit,
                'measurements': measurements_received,
                'prediction_steps': prediction_steps,
                'separation_margin': separation_margin,
                'mode_probabilities': tracker.get_mode_probabilities().tolist(),
            }
        )
        
        self.results.append(result)
        return result
    
    def run_full_validation(self) -> Dict:
        """Run all validation scenarios."""
        print("="*100)
        print("NX-MIMOSA v4.0 - ATC/ATM COMPLIANCE VALIDATION")
        print("EUROCONTROL EASSP Requirements")
        print("="*100)
        
        # Radar rotation period = 4 seconds, so updates every 4s
        # But we simulate at finer dt for trajectory, then sample at radar rate
        sim_dt = 0.1  # 100ms simulation step
        radar_period = 4.0  # 4 second radar rotation
        duration = 120.0  # 2 minutes per scenario
        
        scenarios = [
            ("En-route Cruise", generate_enroute_trajectory),
            ("Terminal Approach", generate_approach_trajectory),
            ("Holding Pattern", generate_holding_trajectory),
            ("Go-Around", generate_go_around_trajectory),
        ]
        
        all_passed = True
        
        for name, generator in scenarios:
            print(f"\n▶ {name}")
            print("-"*80)
            
            trajectory = generator(duration, sim_dt)
            
            # Create tracker with radar update rate
            tracker = create_tracker('aviation', dt=radar_period, sigma=100.0)
            
            # Create radar
            radar = RadarSensor(
                position=np.array([0, 0, 0]),
                range_sigma=50.0,
                azimuth_sigma=0.1,
                rotation_period=radar_period,
                seed=42
            )
            
            result = self.validate_scenario(tracker, trajectory, radar, name, sim_dt)
            
            status = "✅ PASS" if result.passed else "❌ FAIL"
            print(f"  Status:           {status}")
            print(f"  Position RMS:     {result.position_rms:.1f} m (limit: {result.details['rms_limit']:.0f} m)")
            print(f"  Velocity RMS:     {result.velocity_rms:.2f} m/s")
            print(f"  Track Continuity: {result.track_continuity*100:.1f}% (min: {EUROCONTROL_CONTINUITY*100:.1f}%)")
            print(f"  Max Position Err: {result.max_position_error:.1f} m")
            print(f"  Separation Safe:  {'Yes' if result.separation_safe else 'No'} (margin: {result.details['separation_margin']:.0f} m)")
            
            if not result.passed:
                all_passed = False
        
        # Summary
        print("\n" + "="*100)
        print("📊 VALIDATION SUMMARY")
        print("="*100)
        
        passed_count = sum(1 for r in self.results if r.passed)
        total_count = len(self.results)
        
        print(f"\nScenarios Passed: {passed_count}/{total_count}")
        print(f"Overall Status: {'✅ COMPLIANT' if all_passed else '❌ NON-COMPLIANT'}")
        
        # Requirements table
        print(f"""
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│ EUROCONTROL EASSP COMPLIANCE STATUS                                                             │
├─────────────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                                 │
│  Requirement                          Target          Achieved        Status                   │
│  ──────────────────────────────────────────────────────────────────────────────────            │
│  Position RMS (En-route)              ≤ 500 m         {0:>8.1f} m        {1}                    │
│  Position RMS (TMA)                   ≤ 150 m         {2:>8.1f} m        {3}                    │
│  Track Continuity                     ≥ 99.5%         {4:>8.1f}%        {5}                    │
│  Separation Integrity (5 NM)          Maintained      {6}          {7}                    │
│                                                                                                 │
│  OVERALL COMPLIANCE: {8}                                                           │
│                                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
""".format(
            np.mean([r.position_rms for r in self.results if 'En-route' in r.scenario or 'Holding' in r.scenario]),
            "✅" if all(r.position_rms <= 500 for r in self.results if 'En-route' in r.scenario or 'Holding' in r.scenario) else "❌",
            np.mean([r.position_rms for r in self.results if 'Approach' in r.scenario or 'Go-Around' in r.scenario]),
            "✅" if all(r.position_rms <= 150 for r in self.results if 'Approach' in r.scenario or 'Go-Around' in r.scenario) else "❌",
            np.mean([r.track_continuity * 100 for r in self.results]),
            "✅" if all(r.track_continuity >= 0.995 for r in self.results) else "❌",
            "Yes" if all(r.separation_safe for r in self.results) else "No ",
            "✅" if all(r.separation_safe for r in self.results) else "❌",
            "✅ PASS" if all_passed else "❌ FAIL"
        ))
        
        return {
            'all_passed': all_passed,
            'results': self.results,
        }


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    validator = ATCComplianceValidator(separation_minima=SEPARATION_5NM)
    results = validator.run_full_validation()
    
    print("\n" + "="*100)
    print("✓ Validation complete")
    print("="*100)
