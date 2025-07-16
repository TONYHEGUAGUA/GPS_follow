from dronekit import connect, VehicleMode, LocationGlobalRelative
import socket
#from __future__ import print_function
import time
import sys
import serial
import serial.tools.list_ports

# ========== GPS 解析函数 ==========
def parse_nmea(line):
    try:
        if '*' not in line:
            return None
        checksum_reported = line.split('*')[-1].strip()
        data_part = line.split('$')[-1].split('*')[0]
        checksum_calculated = 0
        for char in data_part:
            checksum_calculated ^= ord(char)
        if f"{checksum_calculated:02X}" != checksum_reported:
            return None
        data_type = data_part[:5]
        fields = data_part[6:].split(',')
        result = {"type": data_type, "valid": True}
        if data_type == "GNGGA":
            result.update({
                "time": fields[0] if fields[0] else None,
                "lat": convert_coord(fields[1], fields[2]) if fields[1] else None,
                "lon": convert_coord(fields[3], fields[4]) if fields[3] else None,
                "quality": int(fields[5]) if fields[5] else 0,
                "satellites": int(fields[6]) if fields[6] else 0,
                "hdop": float(fields[7]) if fields[7] else 99.99,
                "altitude": float(fields[8]) if fields[8] else None
            })
        elif data_type == "GNRMC":
            result.update({
                "time": fields[0] if fields[0] else None,
                "status": fields[1],
                "lat": convert_coord(fields[2], fields[3]) if fields[2] else None,
                "lon": convert_coord(fields[4], fields[5]) if fields[4] else None,
                "speed_knots": float(fields[6]) if fields[6] else 0.0,
                "true_course": float(fields[7]) if fields[7] else 0.0,
                "date": fields[8] if fields[8] else None,
                "mag_var": float(fields[9]) if fields[9] else 0.0
            })
        return result
    except Exception as e:
        print(f"解析错误: {e}")
        return None

def convert_coord(value, direction):
    try:
        if not value:
            return None
        degrees = float(value[:2]) if direction in ['N', 'S'] else float(value[:3])
        minutes = float(value[2:]) if direction in ['N', 'S'] else float(value[3:])
        coord = degrees + minutes/60.0
        return -coord if direction in ['S', 'W'] else coord
    except:
        return None

# ========== 无人机连接 ==========
# 请根据实际情况修改连接字符串，如 '127.0.0.1:14550' 或 '/dev/ttyAMA0', baud=57600
vehicle = connect("udp:192.168.4.2:14550", wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    
    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# ========== GPS 串口参数 ==========
GPS_PORT = 'COM12'  # 根据实际情况修改
GPS_BAUD = 38400
GPS_TIMEOUT = 1

try:
    # Arm and take off to altitude of 5 meters
    arm_and_takeoff(5)
    print("Drone is airborne. Starting GPS tracking...")
    with serial.Serial(GPS_PORT, GPS_BAUD, timeout=GPS_TIMEOUT) as ser:
        while True:
            if vehicle.mode.name != "GUIDED":
                print("User has changed flight modes - aborting follow-me")
                break
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line.startswith('$') or len(line) < 6:
                continue
            parsed = parse_nmea(line)
            if parsed and parsed.get('type') == 'GNGGA' and parsed.get('quality', 0) >= 1:
                lat = parsed.get('lat')
                lon = parsed.get('lon')
                if lat is not None and lon is not None:
                    altitude = 6  # meters
                    dest = LocationGlobalRelative(lat, lon, altitude)
                    print(f"Going to: lat={lat:.6f}, lon={lon:.6f}, alt={altitude}")
                    vehicle.simple_goto(dest)
                    time.sleep(1)  # 1秒更新一次
except KeyboardInterrupt:
    print("\n程序终止")
except Exception as e:
    print(f"发生错误: {e}")
finally:
    print("Close vehicle object")
    vehicle.close()
    print("Completed")