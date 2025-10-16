#!/usr/bin/env python3
"""
Vehicle Configuration Generator for Global Race Trajectory Optimization

This script helps users create custom vehicle parameter files (.ini) for the
global trajectory planner with an interactive interface and validation.

Usage:
    python3 vehicle_config_generator.py
    # Or with CLI arguments:
    python3 vehicle_config_generator.py --mass 3.5 --width 0.3 --length 0.5

Author: F1TENTH DAWGS Team
Date: 2025
"""

import configparser
import argparse
import os
import json
from typing import Dict, Any
import numpy as np


class VehicleConfigGenerator:
    """Interactive vehicle configuration generator for global planner"""

    def __init__(self):
        self.config = configparser.ConfigParser()
        self.vehicle_data = {}

    def calculate_defaults(self, mass: float, length: float, width: float) -> Dict[str, Any]:
        """
        Calculate default parameters based on basic vehicle measurements.

        Args:
            mass: Vehicle mass in kg
            length: Vehicle length in m
            width: Vehicle width in m

        Returns:
            Dictionary of calculated default parameters
        """
        # Nominal vertical force per tire
        f_z0 = mass * 9.81 / 4

        # Yaw inertia approximation (rectangular prism)
        I_z = mass * (length**2 + width**2) / 12

        # Drag coefficient approximation (scale model)
        # Based on frontal area and typical RC car drag
        frontal_area = width * 0.15  # Approximate height
        dragcoeff = 0.5 * 1.225 * 0.4 * frontal_area  # c_w ≈ 0.4 for RC cars

        # Maximum forces estimation (aggressive but safe)
        # Assume max acceleration: 10 m/s² forward, 14 m/s² braking
        f_drive_max = mass * 10.0
        f_brake_max = mass * 14.0

        # Maximum power estimation
        # P = F * v, assume max force at v_max/3
        v_max_assumed = 15.0  # m/s
        power_max = f_drive_max * (v_max_assumed / 3)

        return {
            "f_z0": round(f_z0, 2),
            "I_z": round(I_z, 5),
            "dragcoeff": round(dragcoeff, 4),
            "f_drive_max": round(f_drive_max, 1),
            "f_brake_max": round(f_brake_max, 1),
            "power_max": round(power_max, 1)
        }

    def get_tire_preset(self, tire_type: str) -> Dict[str, float]:
        """
        Get Magic Formula parameters for common tire types.

        Args:
            tire_type: One of 'soft_slick', 'hard_slick', 'foam', 'plastic', 'drift'

        Returns:
            Dictionary of tire parameters (B, C, E, c_roll)
        """
        presets = {
            'soft_slick': {
                'B': 10.0,
                'C': 1.4,
                'E': 0.85,
                'c_roll': 0.008,
                'description': 'High-grip racing slicks (soft compound)'
            },
            'hard_slick': {
                'B': 7.4,
                'C': 1.2,
                'E': 0.90,
                'c_roll': 0.010,
                'description': 'Durable racing slicks (hard compound)'
            },
            'foam': {
                'B': 6.0,
                'C': 1.15,
                'E': 0.88,
                'c_roll': 0.012,
                'description': 'Foam tires (indoor carpet)'
            },
            'plastic': {
                'B': 5.0,
                'C': 1.0,
                'E': 0.95,
                'c_roll': 0.015,
                'description': 'Low-grip plastic/ABS tires (drift)'
            },
            'drift': {
                'B': 4.5,
                'C': 1.0,
                'E': 1.0,
                'c_roll': 0.015,
                'description': 'Drift-spec tires (controllable oversteer)'
            }
        }

        return presets.get(tire_type, presets['hard_slick'])

    def interactive_config(self):
        """Interactive terminal-based configuration"""
        print("\n" + "="*70)
        print("🏎️  Global Planner Vehicle Configuration Generator")
        print("="*70)

        # Step 1: Basic measurements
        print("\n📏 STEP 1: Basic Vehicle Measurements")
        print("-" * 70)

        try:
            mass = float(input("Vehicle mass [kg] (e.g., 3.518): ").strip() or "3.518")
            length = float(input("Vehicle length [m] (e.g., 0.535): ").strip() or "0.535")
            width = float(input("Vehicle width [m] (e.g., 0.30): ").strip() or "0.30")
            v_max = float(input("Maximum speed [m/s] (e.g., 15.0): ").strip() or "15.0")
        except ValueError:
            print("❌ Invalid input. Using default values.")
            mass, length, width, v_max = 3.518, 0.535, 0.30, 15.0

        # Calculate defaults
        defaults = self.calculate_defaults(mass, length, width)
        print(f"\n✅ Calculated defaults:")
        print(f"  - Vertical force per tire: {defaults['f_z0']:.2f} N")
        print(f"  - Yaw inertia: {defaults['I_z']:.5f} kg·m²")
        print(f"  - Drag coefficient: {defaults['dragcoeff']:.4f}")

        # Step 2: Tire selection
        print("\n🛞 STEP 2: Tire Configuration")
        print("-" * 70)
        print("Available tire presets:")
        print("  1. Soft Racing Slicks (high grip, racing)")
        print("  2. Hard Racing Slicks (durable, practice)")
        print("  3. Foam Tires (indoor carpet)")
        print("  4. Plastic/ABS (low grip, drifting)")
        print("  5. Drift Spec (controllable oversteer)")
        print("  6. Custom (manual input)")

        tire_choice = input("\nSelect tire preset [1-6] (default: 2): ").strip() or "2"

        tire_map = {
            '1': 'soft_slick',
            '2': 'hard_slick',
            '3': 'foam',
            '4': 'plastic',
            '5': 'drift'
        }

        if tire_choice in tire_map:
            tire_params = self.get_tire_preset(tire_map[tire_choice])
            print(f"\n✅ Selected: {tire_params['description']}")
            print(f"  - Stiffness (B): {tire_params['B']}")
            print(f"  - Shape (C): {tire_params['C']}")
            print(f"  - Curvature (E): {tire_params['E']}")
        else:
            print("\n🔧 Custom tire configuration:")
            try:
                B = float(input("  Stiffness B (4-12, default 7.4): ").strip() or "7.4")
                C = float(input("  Shape C (1.0-1.6, default 1.2): ").strip() or "1.2")
                E = float(input("  Curvature E (0.8-1.0, default 0.85): ").strip() or "0.85")
                c_roll = float(input("  Rolling resistance (0.008-0.02, default 0.01): ").strip() or "0.01")
                tire_params = {'B': B, 'C': C, 'E': E, 'c_roll': c_roll}
            except ValueError:
                print("❌ Invalid input. Using hard slick defaults.")
                tire_params = self.get_tire_preset('hard_slick')

        # Step 3: Friction coefficient
        print("\n🏁 STEP 3: Track Surface")
        print("-" * 70)
        print("Surface friction coefficient (μ):")
        print("  1. Indoor smooth floor (0.7)")
        print("  2. Indoor concrete (0.9)")
        print("  3. Asphalt (1.0)")
        print("  4. Rubber track (1.2)")
        print("  5. Custom")

        surface_choice = input("\nSelect surface [1-5] (default: 3): ").strip() or "3"
        surface_mue = {
            '1': 0.7,
            '2': 0.9,
            '3': 1.0,
            '4': 1.2
        }

        if surface_choice in surface_mue:
            mue = surface_mue[surface_choice]
        else:
            try:
                mue = float(input("Custom friction coefficient (0.5-1.5): ").strip() or "1.0")
            except ValueError:
                mue = 1.0

        print(f"✅ Selected friction: μ = {mue}")

        # Step 4: Drivetrain configuration
        print("\n⚙️  STEP 4: Drivetrain Configuration")
        print("-" * 70)
        print("Drivetrain type:")
        print("  1. Rear-Wheel Drive (RWD)")
        print("  2. Front-Wheel Drive (FWD)")
        print("  3. All-Wheel Drive (AWD)")

        drivetrain = input("\nSelect drivetrain [1-3] (default: 1): ").strip() or "1"

        k_drive_front = {
            '1': 0.0,  # RWD
            '2': 1.0,  # FWD
            '3': 0.5   # AWD
        }.get(drivetrain, 0.0)

        # Step 5: Wheelbase and geometry
        print("\n📐 STEP 5: Vehicle Geometry (Optional)")
        print("-" * 70)
        print("If measured, enter wheelbase values. Otherwise, press Enter for estimates.")

        try:
            wb_front_input = input(f"Front wheelbase [m] (default: {length*0.48:.3f}): ").strip()
            wb_rear_input = input(f"Rear wheelbase [m] (default: {length*0.52:.3f}): ").strip()

            wheelbase_front = float(wb_front_input) if wb_front_input else length * 0.48
            wheelbase_rear = float(wb_rear_input) if wb_rear_input else length * 0.52

            track_width_input = input(f"Track width [m] (default: {width*0.9:.3f}): ").strip()
            track_width = float(track_width_input) if track_width_input else width * 0.9

            cog_z_input = input(f"CoG height [m] (default: {length*0.14:.3f}): ").strip()
            cog_z = float(cog_z_input) if cog_z_input else length * 0.14

            delta_max_input = input("Max steering angle [rad] (default: 0.34 ≈ 19.5°): ").strip()
            delta_max = float(delta_max_input) if delta_max_input else 0.34

        except ValueError:
            print("❌ Invalid input. Using estimated values.")
            wheelbase_front = length * 0.48
            wheelbase_rear = length * 0.52
            track_width = width * 0.9
            cog_z = length * 0.14
            delta_max = 0.34

        # Step 6: File name
        print("\n💾 STEP 6: Save Configuration")
        print("-" * 70)
        filename = input("Configuration filename (without .ini, e.g., 'my_vehicle'): ").strip()
        if not filename:
            filename = f"vehicle_{int(mass*100)}"

        filename = filename if filename.endswith('.ini') else f"{filename}.ini"

        # Build configuration dictionary
        self.vehicle_data = {
            # General options
            'ggv_file': 'ggv.csv',
            'ax_max_machines_file': 'ax_max_machines.csv',
            'stepsize_opts': {
                'stepsize_prep': 0.05,
                'stepsize_reg': 0.2,
                'stepsize_interp_after_opt': 0.1
            },
            'reg_smooth_opts': {
                'k_reg': 3,
                's_reg': 1
            },
            'curv_calc_opts': {
                'd_preview_curv': 2.0,
                'd_review_curv': 2.0,
                'd_preview_head': 1.0,
                'd_review_head': 1.0
            },
            'veh_params': {
                'v_max': v_max,
                'length': length,
                'width': width,
                'mass': mass,
                'dragcoeff': defaults['dragcoeff'],
                'curvlim': 1.0,
                'g': 9.81
            },
            'vel_calc_opts': {
                'dyn_model_exp': 1.0,
                'vel_profile_conv_filt_window': None
            },
            # Optimization options
            'optim_opts_shortest_path': {
                'width_opt': width + 0.1
            },
            'optim_opts_mincurv': {
                'width_opt': width + 0.1,
                'iqp_iters_min': 5,
                'iqp_curverror_allowed': 0.1
            },
            'optim_opts_mintime': {
                'width_opt': width + 0.1,
                'penalty_delta': 1.0,
                'penalty_F': 0.1,
                'mue': mue,
                'n_gauss': 5,
                'dn': 0.025,
                'limit_energy': False,
                'energy_limit': 2.0,
                'safe_traj': False,
                'ax_pos_safe': None,
                'ax_neg_safe': None,
                'ay_safe': None,
                'w_tr_reopt': 1.0,
                'w_veh_reopt': width + 0.1,
                'w_add_spl_regr': 0.0,
                'step_non_reg': 0,
                'eps_kappa': 1e-3
            },
            'vehicle_params_mintime': {
                'wheelbase_front': wheelbase_front,
                'wheelbase_rear': wheelbase_rear,
                'track_width_front': track_width,
                'track_width_rear': track_width,
                'cog_z': cog_z,
                'I_z': defaults['I_z'],
                'liftcoeff_front': 0.001,
                'liftcoeff_rear': 0.0015,
                'k_brake_front': 0.5,
                'k_drive_front': k_drive_front,
                'k_roll': 0.5,
                't_delta': 0.1,
                't_drive': 0.1,
                't_brake': 0.1,
                'power_max': defaults['power_max'],
                'f_drive_max': defaults['f_drive_max'],
                'f_brake_max': defaults['f_brake_max'],
                'delta_max': delta_max
            },
            'tire_params_mintime': {
                'c_roll': tire_params['c_roll'],
                'f_z0': defaults['f_z0'],
                'B_front': tire_params['B'],
                'C_front': tire_params['C'],
                'eps_front': -0.1,
                'E_front': tire_params['E'],
                'B_rear': tire_params['B'],
                'C_rear': tire_params['C'],
                'eps_rear': -0.1,
                'E_rear': tire_params['E']
            },
            'pwr_params_mintime': {
                'pwr_behavior': False,
                'simple_loss': True,
                # Simplified powertrain (not used when pwr_behavior=False)
            }
        }

        # Generate .ini file
        self.generate_ini_file(filename)

        print("\n" + "="*70)
        print(f"✅ Configuration saved to: {filename}")
        print("="*70)
        print("\n📝 Summary:")
        print(f"  Mass: {mass} kg")
        print(f"  Size: {length} x {width} m")
        print(f"  Max speed: {v_max} m/s ({v_max*3.6:.1f} km/h)")
        print(f"  Friction: μ = {mue}")
        print(f"  Tire: B={tire_params['B']}, C={tire_params['C']}, E={tire_params['E']}")
        print(f"  Drivetrain: {'RWD' if k_drive_front == 0 else 'FWD' if k_drive_front == 1 else 'AWD'}")

        print("\n🚀 Next steps:")
        print(f"  1. Edit main_globaltraj.py:")
        print(f"     file_paths['veh_params_file'] = '{filename}'")
        print(f"  2. Run: python3 main_globaltraj.py")
        print(f"  3. Test generated trajectory on vehicle")
        print(f"  4. Tune parameters based on real performance")

        return filename

    def generate_ini_file(self, filename: str):
        """Generate .ini configuration file"""

        # Get script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        params_dir = os.path.join(script_dir, 'params')
        os.makedirs(params_dir, exist_ok=True)

        filepath = os.path.join(params_dir, filename)

        # Create ConfigParser
        config = configparser.ConfigParser()

        # Add sections
        config['GENERAL_OPTIONS'] = {}
        gen_opts = config['GENERAL_OPTIONS']

        gen_opts['ggv_file'] = json.dumps(self.vehicle_data['ggv_file'])
        gen_opts['ax_max_machines_file'] = json.dumps(self.vehicle_data['ax_max_machines_file'])
        gen_opts['stepsize_opts'] = json.dumps(self.vehicle_data['stepsize_opts'])
        gen_opts['reg_smooth_opts'] = json.dumps(self.vehicle_data['reg_smooth_opts'])
        gen_opts['curv_calc_opts'] = json.dumps(self.vehicle_data['curv_calc_opts'])
        gen_opts['veh_params'] = json.dumps(self.vehicle_data['veh_params'])
        gen_opts['vel_calc_opts'] = json.dumps(self.vehicle_data['vel_calc_opts'])

        config['OPTIMIZATION_OPTIONS'] = {}
        opt_opts = config['OPTIMIZATION_OPTIONS']

        opt_opts['optim_opts_shortest_path'] = json.dumps(self.vehicle_data['optim_opts_shortest_path'])
        opt_opts['optim_opts_mincurv'] = json.dumps(self.vehicle_data['optim_opts_mincurv'])
        opt_opts['optim_opts_mintime'] = json.dumps(self.vehicle_data['optim_opts_mintime'])
        opt_opts['vehicle_params_mintime'] = json.dumps(self.vehicle_data['vehicle_params_mintime'])
        opt_opts['tire_params_mintime'] = json.dumps(self.vehicle_data['tire_params_mintime'])
        opt_opts['pwr_params_mintime'] = json.dumps({'pwr_behavior': False, 'simple_loss': True})

        # Write to file
        with open(filepath, 'w') as f:
            f.write("# Vehicle configuration generated by vehicle_config_generator.py\n")
            f.write("# " + "="*70 + "\n\n")
            config.write(f)

        print(f"\n✅ File written: {filepath}")


def parse_cli_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(
        description='Generate vehicle configuration for global trajectory planner'
    )

    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Run in interactive mode (default)')
    parser.add_argument('--mass', type=float,
                       help='Vehicle mass in kg')
    parser.add_argument('--length', type=float,
                       help='Vehicle length in m')
    parser.add_argument('--width', type=float,
                       help='Vehicle width in m')
    parser.add_argument('--v-max', type=float, dest='v_max',
                       help='Maximum speed in m/s')
    parser.add_argument('--mue', type=float,
                       help='Friction coefficient (0.5-1.5)')
    parser.add_argument('--tire-preset', type=str,
                       choices=['soft_slick', 'hard_slick', 'foam', 'plastic', 'drift'],
                       help='Tire preset type')
    parser.add_argument('--output', '-o', type=str,
                       help='Output filename (without .ini extension)')

    return parser.parse_args()


def main():
    """Main entry point"""
    args = parse_cli_args()

    generator = VehicleConfigGenerator()

    # If no CLI args provided or interactive flag set, run interactive mode
    if args.interactive or not any([args.mass, args.length, args.width]):
        generator.interactive_config()
    else:
        # TODO: Implement non-interactive mode with CLI args
        print("Non-interactive mode not yet implemented. Running interactive mode...")
        generator.interactive_config()


if __name__ == '__main__':
    main()
