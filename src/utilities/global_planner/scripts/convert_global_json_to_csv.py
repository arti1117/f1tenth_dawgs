#!/usr/bin/env python3
"""
convert_global_json_to_csv.py
Usage:
  python3 convert_global_json_to_csv.py /path/to/global_waypoints.json -o out.csv --which iqp
"""
import json, csv, argparse, sys, os

def extract_wpnts(obj):
    if obj is None:
        return []
    if isinstance(obj, dict):
        if 'wpnts' in obj:
            return obj['wpnts']
        # sometimes direct list
    if isinstance(obj, list):
        return obj
    return []

def unwrap_field(val):
    # handle { "data": 1.23 } or plain number
    if isinstance(val, dict) and 'data' in val:
        return val['data']
    return val

def main():
    p = argparse.ArgumentParser()
    p.add_argument('jsonfile', help='global_waypoints.json path')
    p.add_argument('-o','--out', default='global_xyvk.csv', help='output csv path')
    p.add_argument('--which', choices=['iqp','sp','auto'], default='auto', help='choose trajectory (iqp or sp)')
    args = p.parse_args()

    with open(args.jsonfile, 'r') as f:
        d = json.load(f)

    if args.which == 'iqp':
        key = 'global_traj_wpnts_iqp'
    elif args.which == 'sp':
        key = 'global_traj_wpnts_sp'
    else:
        key = 'global_traj_wpnts_iqp' if 'global_traj_wpnts_iqp' in d else ('global_traj_wpnts_sp' if 'global_traj_wpnts_sp' in d else None)

    if key is None:
        print("ERROR: No global_traj_wpnts_iqp or global_traj_wpnts_sp found in JSON", file=sys.stderr)
        sys.exit(1)

    arr = extract_wpnts(d[key])
    if not arr:
        print("ERROR: empty waypoint list", file=sys.stderr)
        sys.exit(1)

    # Write CSV header: id, s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
    with open(args.out, 'w', newline='') as csvf:
        w = csv.writer(csvf)
        w.writerow(['id','s_m','x_m','y_m','psi_rad','kappa_radpm','vx_mps','ax_mps2'])
        for wp in arr:
            # some fields may be nested dicts with 'data', handle both
            def gv(name):
                v = wp.get(name)
                return unwrap_field(v) if v is not None else ''
            row = [gv('id'), gv('s_m'), gv('x_m'), gv('y_m'), gv('psi_rad'), gv('kappa_radpm'), gv('vx_mps'), gv('ax_mps2')]
            w.writerow(row)

    print(f"Wrote {args.out} ({len(arr)} points)")

if __name__ == "__main__":
    main()
