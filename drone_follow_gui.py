import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import serial
import time

# 这里假设你已将parse_nmea、convert_coord、arm_and_takeoff等函数复制到本文件
# 并已安装dronekit、pyserial等依赖
from dronekit import connect, VehicleMode, LocationGlobalRelative

# ========== GPS 解析函数（可从drone_follow.py复制过来） ==========
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

def arm_and_takeoff(vehicle, aTargetAltitude, log_func):
    log_func("Basic pre-arm checks")
    while not vehicle.is_armable:
        log_func(" Waiting for vehicle to initialise...")
        time.sleep(1)
    log_func("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    
    while not vehicle.armed:
        log_func(" Waiting for arming...")
        time.sleep(1)
    log_func("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        log_func(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            log_func("Reached target altitude")
            break
        time.sleep(1)

# ========== 追踪线程 ==========
class TrackerThread(threading.Thread):
    def __init__(self, port, baud, vehicle, log_func, stop_event):
        super().__init__()
        self.port = port
        self.baud = baud
        self.vehicle = vehicle
        self.log_func = log_func
        self.stop_event = stop_event

    def run(self):
        try:
            self.log_func("开始GPS追踪...")
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                while not self.stop_event.is_set():
                    if self.vehicle.mode.name != "GUIDED":
                        self.log_func("User has changed flight modes - aborting follow-me")
                        break
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if not line.startswith('$') or len(line) < 6:
                        continue
                    parsed = parse_nmea(line)
                    if parsed and parsed.get('type') == 'GNGGA' and parsed.get('quality', 0) >= 1:
                        lat = parsed.get('lat')
                        lon = parsed.get('lon')
                        if lat is not None and lon is not None:
                            altitude = self.vehicle.location.global_relative_frame.alt or 6
                            dest = LocationGlobalRelative(lat, lon, altitude)
                            self.log_func(f"Going to: lat={lat:.6f}, lon={lon:.6f}, alt={altitude}")
                            self.vehicle.simple_goto(dest)
                            time.sleep(1)
        except Exception as e:
            self.log_func(f"发生错误: {e}")
        finally:
            self.log_func("追踪线程已结束")

# ========== UI界面 ==========
class DroneFollowGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("无人机GPS追踪控制台")
        self.root.geometry("420x360")

        # 串口选择
        ttk.Label(root, text="串口:").place(x=20, y=20)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(root, textvariable=self.port_var, width=15)
        self.port_combo.place(x=70, y=20)
        self.refresh_ports()
        ttk.Button(root, text="刷新", command=self.refresh_ports).place(x=220, y=18)

        # 波特率选择
        ttk.Label(root, text="波特率:").place(x=20, y=60)
        self.baud_var = tk.StringVar(value="38400")
        self.baud_combo = ttk.Combobox(root, textvariable=self.baud_var, width=15, values=["9600","19200","38400","57600","115200"])
        self.baud_combo.place(x=70, y=60)

        # 无人机连接字符串
        ttk.Label(root, text="无人机连接:").place(x=20, y=100)
        self.conn_var = tk.StringVar(value="udp:192.168.4.2:14550")
        self.conn_entry = ttk.Entry(root, textvariable=self.conn_var, width=18)
        self.conn_entry.place(x=110, y=100)

        # 起飞高度
        ttk.Label(root, text="起飞高度(m):").place(x=20, y=140)
        self.alt_var = tk.StringVar(value="5")
        self.alt_entry = ttk.Entry(root, textvariable=self.alt_var, width=6)
        self.alt_entry.place(x=110, y=140)

        # 按钮
        self.connect_btn = ttk.Button(root, text="连接飞机", command=self.connect_vehicle)
        self.connect_btn.place(x=20, y=180)
        self.takeoff_btn = ttk.Button(root, text="起飞", command=self.takeoff_vehicle, state=tk.DISABLED)
        self.takeoff_btn.place(x=120, y=180)
        self.start_btn = ttk.Button(root, text="开始追踪", command=self.start_tracking, state=tk.DISABLED)
        self.start_btn.place(x=200, y=180)
        self.stop_btn = ttk.Button(root, text="取消追踪", command=self.stop_tracking, state=tk.DISABLED)
        self.stop_btn.place(x=320, y=180)
        self.test_gps_btn = ttk.Button(root, text="测试GPS", command=self.test_gps)
        self.test_gps_btn.place(x=320, y=60)

        # 日志显示
        self.log_text = tk.Text(root, height=8, width=52, state=tk.DISABLED)
        self.log_text.place(x=10, y=230)

        self.tracker_thread = None
        self.stop_event = threading.Event()
        self.vehicle = None
        self.vehicle_connected = False
        self.vehicle_armed = False
        self.vehicle_airborne = False

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def log(self, msg):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg + '\n')
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def connect_vehicle(self):
        conn = self.conn_var.get()
        if not conn:
            messagebox.showerror("错误", "请填写无人机连接参数！")
            return
        self.log("正在连接无人机...（请稍候）")
        self.connect_btn.config(state=tk.DISABLED)
        def do_connect():
            try:
                vehicle = connect(conn, wait_ready=True, timeout=300)
                def on_success():
                    self.vehicle = vehicle
                    self.vehicle_connected = True
                    self.log("无人机连接成功！")
                    self.connect_btn.config(state=tk.DISABLED)
                    self.takeoff_btn.config(state=tk.NORMAL)
                self.root.after(0, on_success)
            except Exception as e:
                def on_fail(e=e):
                    self.log(f"连接无人机失败: {e}")
                    self.vehicle = None
                    self.vehicle_connected = False
                    self.connect_btn.config(state=tk.NORMAL)
                self.root.after(0, on_fail)
        threading.Thread(target=do_connect, daemon=True).start()

    def takeoff_vehicle(self):
        if not self.vehicle or not self.vehicle_connected:
            self.log("请先连接无人机！")
            return
        try:
            target_alt = float(self.alt_var.get())
        except Exception:
            self.log("请输入有效的起飞高度！")
            return
        self.log("开始起飞...（请稍候）")
        self.takeoff_btn.config(state=tk.DISABLED)
        def do_takeoff():
            try:
                if not self.vehicle:
                    def on_fail():
                        self.log("起飞失败：无人机对象不存在！")
                        self.takeoff_btn.config(state=tk.NORMAL)
                    self.root.after(0, on_fail)
                    return
                self.log("Basic pre-arm checks")
                timeout = 15  # 最多等待15秒
                waited = 0
                while not self.vehicle.is_armable and waited < timeout:
                    self.log(" Waiting for vehicle to initialise...")
                    time.sleep(1)
                    waited += 1
                if not self.vehicle.is_armable:
                    def on_fail():
                        self.log("无法解锁：无人机未准备好（is_armable=False）")
                        self.takeoff_btn.config(state=tk.NORMAL)
                    self.root.after(0, on_fail)
                    return
                self.log("Arming motors")
                self.vehicle.mode = VehicleMode("GUIDED")
                self.vehicle.armed = True
                waited = 0
                while not self.vehicle.armed and waited < timeout:
                    self.log(" Waiting for arming...")
                    time.sleep(1)
                    waited += 1
                if not self.vehicle.armed:
                    def on_fail():
                        self.log("无法解锁：无人机未成功解锁（armed=False）")
                        self.takeoff_btn.config(state=tk.NORMAL)
                    self.root.after(0, on_fail)
                    return
                self.log("Taking off!")
                if not hasattr(self.vehicle, 'simple_takeoff'):
                    def on_fail():
                        self.log("起飞失败：无人机对象不支持simple_takeoff方法！")
                        self.takeoff_btn.config(state=tk.NORMAL)
                    self.root.after(0, on_fail)
                    return
                self.vehicle.simple_takeoff(target_alt)
                # 等待到达目标高度
                while True:
                    if not self.vehicle or not hasattr(self.vehicle, 'location') or not hasattr(self.vehicle.location, 'global_relative_frame'):
                        def on_fail():
                            self.log("起飞失败：无法获取无人机高度信息！")
                            self.takeoff_btn.config(state=tk.NORMAL)
                        self.root.after(0, on_fail)
                        return
                    alt = self.vehicle.location.global_relative_frame.alt
                    self.log(f" Altitude: {alt}")
                    if alt >= target_alt * 0.95:
                        break
                    time.sleep(1)
                def on_success():
                    self.vehicle_armed = True
                    self.vehicle_airborne = True
                    self.start_btn.config(state=tk.NORMAL)
                    self.log("起飞成功，准备追踪！")
                self.root.after(0, on_success)
            except Exception as e:
                def on_fail():
                    self.log(f"起飞失败: {e}")
                    self.takeoff_btn.config(state=tk.NORMAL)
                self.root.after(0, on_fail)
        threading.Thread(target=do_takeoff, daemon=True).start()

    def start_tracking(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port or not baud or not self.vehicle or not self.vehicle_connected or not self.vehicle_airborne:
            messagebox.showerror("错误", "请确保已连接无人机并起飞！")
            return
        self.stop_event.clear()
        self.tracker_thread = TrackerThread(port, baud, self.vehicle, self.log, self.stop_event)
        self.tracker_thread.start()
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.log("追踪已启动...")

    def stop_tracking(self):
        self.stop_event.set()
        self.log("正在停止追踪...")
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def test_gps(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port or not baud:
            messagebox.showerror("错误", "请先选择串口和波特率！")
            return
        self.log("正在测试GPS信号...（请稍候）")
        self.test_gps_btn.config(state=tk.DISABLED)
        def do_test():
            import time
            found = False
            try:
                with serial.Serial(port, baud, timeout=1) as ser:
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        line = ser.readline().decode('ascii', errors='ignore').strip()
                        if not line.startswith('$') or len(line) < 6:
                            continue
                        parsed = parse_nmea(line)
                        if parsed and parsed.get('type') in ('GNGGA', 'GNRMC'):
                            lat = parsed.get('lat')
                            lon = parsed.get('lon')
                            if lat is not None and lon is not None:
                                found = True
                                def on_success(lat=lat, lon=lon):
                                    self.log(f"GPS信号正常，经度: {lon:.6f}，纬度: {lat:.6f}")
                                    self.test_gps_btn.config(state=tk.NORMAL)
                                self.root.after(0, on_success)
                                return
                if not found:
                    def on_fail():
                        self.log("无GPS信号！")
                        self.test_gps_btn.config(state=tk.NORMAL)
                    self.root.after(0, on_fail)
            except Exception as e:
                def on_fail():
                    self.log(f"测试GPS失败: {e}")
                    self.test_gps_btn.config(state=tk.NORMAL)
                self.root.after(0, on_fail)
        threading.Thread(target=do_test, daemon=True).start()

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneFollowGUI(root)
    root.mainloop() 