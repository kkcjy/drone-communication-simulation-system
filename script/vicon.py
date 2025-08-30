import pandas as pd
import motioncapture 
import time
import os
import math


# This script receives rigid body motion data in real-time from the host of the Vicon motion capture system via a network connection. 


def calculate_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2) * 100    # m -> cm

def get_system_time():
    return int(time.monotonic() * 1000)


if __name__ == "__main__":
    df = pd.DataFrame(columns=[
        'rigid_body_name', 'time', 
        'position_x', 'position_y', 'position_z',
        'rotation_x', 'rotation_y', 'rotation_z'
    ])
    
    rigid_cache = dict()
    distance_lines = []  
    
    detection_phase = True      # Whether in the initial detection phase
    detection_duration = 5      # Detection duration (seconds), adjustable
    detected_rigids = set()     # Names of all detected rigid bodies
    target_rigids = []          # Final filtered list of rigid bodies to monitor
    
    output_dir = '../data'
    os.makedirs(output_dir, exist_ok=True)

    # Connect to Vicon host
    mc = motioncapture.connect("vicon", {"hostname": "172.20.10.2"})
    
    try:
        start_time = time.time()
        print(f"Starting initial rigid body detection... (Auto-filtering in {detection_duration} seconds)")
        
        while True:
            current_run_time = time.time() - start_time
            current_time = get_system_time()
            
            # 1. Initial detection phase: Collect all detected rigid body names
            if detection_phase:
                mc.waitForNextFrame()
                # Iterate through all rigid bodies in current frame and record names
                for rigid_body_name in mc.rigidBodies.keys():
                    detected_rigids.add(rigid_body_name)
                
                # End detection phase when duration is reached
                if current_run_time >= detection_duration:
                    detection_phase = False
                    print("\n===== Initial Detection Complete =====")
                    print(f"Detected {len(detected_rigids)} rigid bodies: {sorted(detected_rigids)}")
                    user_input = input("Enter rigid body names to monitor (space-separated, Enter to keep all): ")
                    
                    if user_input.strip():
                        target_rigids = [name.strip() for name in user_input.split()]
                        invalid = [name for name in target_rigids if name not in detected_rigids]
                        if invalid:
                            print(f"Warning: The following rigid bodies were not detected and will be ignored: {invalid}")
                            target_rigids = [name for name in target_rigids if name in detected_rigids]
                    else:
                        target_rigids = sorted(detected_rigids)
                    
                    print(f"Final monitoring list: {target_rigids}")
                    print("\nStarting formal data collection... (Press Ctrl+C to stop)")
            
            # 2. Formal processing phase: Only process filtered rigid bodies
            else:
                mc.waitForNextFrame()
                for rigid_body_name, obj in mc.rigidBodies.items():
                    if rigid_body_name not in target_rigids:
                        continue
                    
                    position = (obj.position[0], obj.position[1], obj.position[2])
                    rigid_cache[rigid_body_name] = {
                        'position': position,
                        'timestamp': current_time,
                        'updated': True
                    }
                
                # Calculate and store distances in specified format
                if len(target_rigids) >= 2:
                    all_updated = all(
                        rigid_cache.get(rigid, {}).get('updated', False) 
                        for rigid in target_rigids
                    )
                    
                    if all_updated:
                        # Generate distance results in required format
                        for i in range(len(target_rigids)):
                            for j in range(i + 1, len(target_rigids)):
                                rigid1 = target_rigids[i]
                                rigid2 = target_rigids[j]
                                
                                pos1 = rigid_cache[rigid1]['position']
                                pos2 = rigid_cache[rigid2]['position']
                                distance = calculate_distance(pos1, pos2)
                                
                                line1 = f"[local_{rigid1} <- neighbor_{rigid2}]: vicon dist = {distance:.4f}, time = {current_time}"
                                line2 = f"[local_{rigid2} <- neighbor_{rigid1}]: vicon dist = {distance:.4f}, time = {current_time}"
                                
                                print(line1)
                                print(line2)
                                
                                distance_lines.append(line1)
                                distance_lines.append(line2)
                    
                    for rigid in target_rigids:
                        if rigid in rigid_cache:
                            rigid_cache[rigid]['updated'] = False
    
    except KeyboardInterrupt:
        print("\nUser interrupted collection, saving distance data...")
    finally:
        if distance_lines:
            distance_file = os.path.join(output_dir, 'vicon.txt')
            with open(distance_file, 'w') as f:
                for line in distance_lines:
                    f.write(f"{line}\n")
            print(f"\nDistance information saved to: {distance_file}")
        
        print("\nData collection complete!")