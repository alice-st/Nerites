#!/usr/bin/env python3
import os
import sys
import numpy as np
import xml.etree.ElementTree as ET

def get_float_list(node):
    """Convert a string of numbers into a list of floats."""
    return list(map(float, node.text.strip().split())) if node is not None and node.text else []

def scan_dae_file(file_path):
    """
    Scan a .dae file and compute the bounding box from its <float_array> positions.
    Return (status, min_corner, max_corner) or (error, None, None).
    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()

        # COLLADA files have namespaces; we need to strip them to find elements easily
        ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}

        # Find all float_array nodes that describe vertex positions
        positions = []
        for source in root.findall(".//c:source", ns):
            float_array = source.find("c:float_array", ns)
            accessor = source.find("c:technique_common/c:accessor", ns)
            if float_array is None or accessor is None:
                continue

            # Only process position data (typically 'POSITION' semantic)
            params = accessor.findall("c:param", ns)
            if params and params[0].attrib.get("name", "").lower() in ["x", "position", "pos"]:
                coords = get_float_list(float_array)
                # group coords into (x,y,z)
                for i in range(0, len(coords), 3):
                    positions.append(coords[i:i+3])

        if not positions:
            return ("No vertex positions found", None, None)

        points = np.array(positions)
        min_corner = points.min(axis=0)
        max_corner = points.max(axis=0)

        # Check for invalid bounding box
        if np.any(np.isnan(min_corner)) or np.any(np.isnan(max_corner)):
            return ("NaN detected in coordinates", min_corner, max_corner)

        if np.any(min_corner > max_corner):
            return ("Broken bounding box (min > max)", min_corner, max_corner)

        return ("OK", min_corner, max_corner)

    except Exception as e:
        return (f"Error reading file: {e}", None, None)

def scan_directory_for_dae(directory):
    """Scan all .dae files in a directory recursively."""
    print(f"🔍 Scanning {directory} for .dae files...\n")
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".dae"):
                path = os.path.join(root, file)
                status, min_corner, max_corner = scan_dae_file(path)
                print(f" {path}")
                print(f"   → {status}")
                if min_corner is not None:
                    print(f"     Min corner: {min_corner}")
                    print(f"     Max corner: {max_corner}")
                print()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 check_dae_bounding_boxes.py <directory_with_dae_files>")
        sys.exit(1)

    scan_directory_for_dae(sys.argv[1])
