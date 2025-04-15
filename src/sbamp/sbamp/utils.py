import csv

def load_waypoints(waypoint_file_path):
    waypoints = []
    try:
        with open(waypoint_file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader) # Skip the header
            for row in reader:
                x = float(row[0])
                y = float(row[1])
                yaw = float(row[2])
                qw = float(row[3])
                qx = float(row[4])
                qy = float(row[5])
                qz = float(row[6])
                waypoints.append((x, y, yaw, qw, qx, qy, qz))
    except Exception as e:
        print(f"Failed to read waypoints: {e}")

    return waypoints