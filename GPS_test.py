import serial
import re
import time
from datetime import datetime

def parse_nmea(line):
    """解析单条NMEA语句"""
    try:
        # 校验和验证
        if '*' not in line:
            return None
            
        checksum_reported = line.split('*')[-1].strip()
        data_part = line.split('$')[-1].split('*')[0]
        checksum_calculated = 0
        
        for char in data_part:
            checksum_calculated ^= ord(char)
        
        if f"{checksum_calculated:02X}" != checksum_reported:
            return None  # 校验失败
        
        # 提取数据类型和字段
        data_type = data_part[:5]
        fields = data_part[6:].split(',')
        
        result = {"type": data_type, "valid": True}
        
        # 根据不同数据类型解析
        if data_type == "GNGGA":  # 全球定位数据
            result.update({
                "time": fields[0] if fields[0] else None,
                "lat": convert_coord(fields[1], fields[2]) if fields[1] else None,
                "lon": convert_coord(fields[3], fields[4]) if fields[3] else None,
                "quality": int(fields[5]) if fields[5] else 0,
                "satellites": int(fields[6]) if fields[6] else 0,
                "hdop": float(fields[7]) if fields[7] else 99.99,
                "altitude": float(fields[8]) if fields[8] else None
            })
            
        elif data_type == "GNRMC":  # 推荐最小定位信息
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
            
        elif data_type in ["GNGSA", "GPGSV", "GAGSV"]:  # 卫星相关数据
            result["sat_info"] = fields
            
        return result
        
    except Exception as e:
        print(f"解析错误: {e}")
        return None

def convert_coord(value, direction):
    """将NMEA坐标转换为十进制度数"""
    try:
        if not value: 
            return None
            
        degrees = float(value[:2]) if direction in ['N', 'S'] else float(value[:3])
        minutes = float(value[2:]) if direction in ['N', 'S'] else float(value[3:])
        coord = degrees + minutes/60.0
        return -coord if direction in ['S', 'W'] else coord
        
    except:
        return None

def main():
    # 配置串口参数 (根据您的GPS模块调整)
    PORT = 'COM12'
    BAUD_RATE = 38400  # 常见NMEA波特率
    TIMEOUT = 1
    
    try:
        # 初始化串口连接
        with serial.Serial(PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            print(f"已连接 {PORT}, 监听中...")
            
            while True:
                try:
                    # 读取一行NMEA数据
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    
                    # 验证基本格式
                    if not line.startswith('$') or len(line) < 6:
                        continue
                    
                    # 解析数据
                    parsed = parse_nmea(line)
                    
                    if parsed:
                        # 打印解析结果
                        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 收到 {parsed['type']}")
                        
                        if parsed.get('time'):
                            gps_time = f"{parsed['time'][0:2]}:{parsed['time'][2:4]}:{parsed['time'][4:6]}"
                            print(f"GPS时间: {gps_time}")
                            
                        if parsed.get('lat') and parsed.get('lon'):
                            print(f"坐标: {parsed['lat']:.6f}°N, {parsed['lon']:.6f}°E")
                            
                        if parsed.get('quality') is not None:
                            status = "有效定位" if parsed['quality'] >= 1 else "无效定位"
                            print(f"状态: {status} | 卫星数: {parsed.get('satellites',0)}")
                    
                    # 避免高CPU占用
                    time.sleep(0.01)
                    
                except KeyboardInterrupt:
                    print("\n程序终止")
                    break
                except Exception as e:
                    print(f"处理错误: {e}")
                    
    except serial.SerialException as e:
        print(f"串口错误: {e}. 请检查:")
        print(f"1. {PORT} 是否存在")
        print(f"2. 波特率设置: 当前 {BAUD_RATE}")
        print(f"3. 其他程序是否占用了串口")

if __name__ == "__main__":
    main()