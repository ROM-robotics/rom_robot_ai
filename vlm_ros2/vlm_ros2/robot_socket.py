import socket
import json
import threading
import time

class RobotCommunication:
    def __init__(self, host='0.0.0.0', port=5000):
        self.host = host
        self.port = port
        self.running = False
        self.data_received = None
        self.lock = threading.Lock() # Thread-safe ဖြစ်အောင် data ကို lock ပေးခြင်း

    def _listen(self):
        """ Server အနေနဲ့ လက်ခံတဲ့ Thread """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()
            while self.running:
                conn, addr = s.accept()
                with conn:
                    while True:
                        data = conn.recv(2048)
                        if not data: break
                        with self.lock:
                            self.data_received = json.loads(data.decode('utf-8'))
    
    def start_receiver(self):
        """ Background မှာ data အမြဲလက်ခံနေရန် """
        self.running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()
        print(f"Receiver started on {self.port}...")

    def send_data(self, target_ip, header, pos=(0.0, 0.0), ori=(0.0, 0.0, 0.0, 1.0)):
        """ Data ပို့ရန် Method """
        payload = {
            "request_header": str(header),
            "position_x": float(pos[0]),
            "position_y": float(pos[1]),
            "orientation_x": float(ori[0]),
            "orientation_y": float(ori[1]),
            "orientation_z": float(ori[2]),
            "orientation_w": float(ori[3])
        }
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1.0) # Connection မရရင် ၁ စက္ကန့်ပဲ စောင့်မယ်
                s.connect((target_ip, self.port))
                s.sendall(json.dumps(payload).encode('utf-8'))
        except Exception as e:
            print(f"Send Error: {e}")

    def get_latest_data(self):
        """ လက်ခံရရှိထားတဲ့ နောက်ဆုံး data ကို ယူဖတ်ရန် (read-and-clear pattern)
        
        Cache problem ဖြေရှင်းဖို့ read လုပ်ပြီးရင် clear လုပ်ပေးတယ်။
        ဒါမှ နောက်ဆုံး command ကို တခါပဲ process လုပ်မယ်။
        """
        with self.lock:
            data = self.data_received
            self.data_received = None  # Clear after reading
            return data

    def stop(self):
        self.running = False