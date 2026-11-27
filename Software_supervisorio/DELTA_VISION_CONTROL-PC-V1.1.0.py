import sys
import os
import json
import time
import threading
import importlib
cv2 = importlib.import_module("cv2")
np = importlib.import_module("numpy")
serial = importlib.import_module("serial")
serial_list_ports = importlib.import_module("serial.tools.list_ports")
_qt_core = importlib.import_module("PyQt5.QtCore")
Qt = _qt_core.Qt
QThread = _qt_core.QThread
pyqtSignal = _qt_core.pyqtSignal
pyqtSlot = _qt_core.pyqtSlot
QTimer = _qt_core.QTimer
_qt_gui = importlib.import_module("PyQt5.QtGui")
QImage = _qt_gui.QImage
QPixmap = _qt_gui.QPixmap
_qt_widgets = importlib.import_module("PyQt5.QtWidgets")
QApplication = _qt_widgets.QApplication
QMainWindow = _qt_widgets.QMainWindow
QWidget = _qt_widgets.QWidget
QLabel = _qt_widgets.QLabel
QPushButton = _qt_widgets.QPushButton
QVBoxLayout = _qt_widgets.QVBoxLayout
QHBoxLayout = _qt_widgets.QHBoxLayout
QGridLayout = _qt_widgets.QGridLayout
QGroupBox = _qt_widgets.QGroupBox
QComboBox = _qt_widgets.QComboBox
QDoubleSpinBox = _qt_widgets.QDoubleSpinBox
QSpinBox = _qt_widgets.QSpinBox
QTextEdit = _qt_widgets.QTextEdit
QTabWidget = _qt_widgets.QTabWidget
QSlider = _qt_widgets.QSlider
QMessageBox = _qt_widgets.QMessageBox
QFrame = _qt_widgets.QFrame
QStackedWidget = _qt_widgets.QStackedWidget
QLineEdit = _qt_widgets.QLineEdit
QListWidget = _qt_widgets.QListWidget
QListWidgetItem = _qt_widgets.QListWidgetItem
QFormLayout = _qt_widgets.QFormLayout
QScrollArea = _qt_widgets.QScrollArea
QCheckBox = _qt_widgets.QCheckBox
QTableWidget = _qt_widgets.QTableWidget
QTableWidgetItem = _qt_widgets.QTableWidgetItem
QHeaderView = _qt_widgets.QHeaderView
QAbstractItemView = _qt_widgets.QAbstractItemView

CONFIG_FILE = "config_delta.json"
HOMOGRAPHY_FILE = "matriz_homografia.npy"
LOGO_FILE = "UEG.png"

# ==============================================================================
# ESTILIZAÇÃO QSS (TEMA AZUL & CINZA INDUSTRIAL)
# ==============================================================================
THEME_QSS = """
QMainWindow { background-color: #1e242b; }
QWidget { background-color: #1e242b; color: #ecf0f1; font-family: 'Segoe UI', sans-serif; font-size: 10pt; }
QGroupBox { border: 1px solid #34495e; border-radius: 6px; margin-top: 10px; font-weight: bold; color: #5dade2; padding-top: 12px; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }
QTabWidget::pane { border: 1px solid #2c3e50; background-color: #242d38; border-radius: 4px; }
QTabBar::tab { background-color: #2c3e50; color: #bdc3c7; padding: 8px 16px; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 2px; }
QTabBar::tab:selected { background-color: #1b4f72; color: #ffffff; font-weight: bold; }
QPushButton { background-color: #2980b9; color: #ffffff; border: none; border-radius: 4px; padding: 6px 12px; font-weight: bold; }
QPushButton:hover { background-color: #3498db; }
QPushButton:pressed { background-color: #1f618d; }
QPushButton#btn_stop { background-color: #c0392b; }
QPushButton#btn_stop:hover { background-color: #e74c3c; }
QPushButton#btn_cycle { background-color: #27ae60; font-size: 11pt; padding: 10px; }
QPushButton#btn_cycle:hover { background-color: #2ecc71; }
QPushButton#btn_conveyor { background-color: #d35400; font-weight: bold; }
QPushButton#btn_conveyor:hover { background-color: #e67e22; }
QComboBox, QDoubleSpinBox, QSpinBox { background-color: #2c3e50; border: 1px solid #34495e; border-radius: 4px; padding: 4px; color: #ecf0f1; }
QTextEdit { background-color: #15191e; border: 1px solid #2c3e50; color: #2ecc71; font-family: 'Consolas', monospace; font-size: 9pt; border-radius: 4px; }
QTextEdit#manual_browser { background-color: #1a2026; border: 1px solid #2c3e50; color: #ecf0f1; font-family: 'Segoe UI', sans-serif; font-size: 10pt; padding: 10px; }
QSlider::groove:horizontal { border: 1px solid #34495e; height: 6px; background: #2c3e50; border-radius: 3px; }
QSlider::sub-page:horizontal { background: #3498db; border-radius: 3px; }
QSlider::handle:horizontal { background: #ecf0f1; border: 1px solid #bdc3c7; width: 14px; margin-top: -4px; margin-bottom: -4px; border-radius: 7px; }
"""

# ==============================================================================
# GERENCIAMENTO DE PERSISTÊNCIA
# ==============================================================================
class ConfigManager:
    @staticmethod
    def load():
        defaults = {
            "port": "",
            "camera_index": 0,
            "z_transit": -550.0,
            "z_pick": -620.0,
            "z_place": -610.0,
            "place_x": 80.0,
            "place_y": 80.0,
            "grid_rows": 2,
            "grid_cols": 2,
            "grid_spacing_x": 40.0,
            "grid_spacing_y": 40.0,
            "hsv": {
                "h_min": 5, "s_min": 100, "v_min": 100,
                "h_max": 22, "s_max": 255, "v_max": 255
            },
            "min_area": 500,
            "max_area": 100000,
            "hsv_profiles": {},
            "vision_tracking": {"stable_frames": 8, "max_match_px": 55, "max_missed_frames": 30, "stable_move_px": 7, "recovery_match_px": 90, "area_tolerance_pct": 45},
            "tool_offset_z": 0.0,
            "z_min_limit": -800.0,
            "holding": {"x": 0.0, "y": 0.0, "z": -550.0, "tool": "OFF"},
            "pick_place": {"approach_z": -550.0, "pick_z": -650.0, "safe_z": -500.0,
                           "drop_x": 150.0, "drop_y": 0.0, "drop_z": -650.0}
        }
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r") as f:
                    data = json.load(f)
                    defaults.update(data)
                    # Compatibilidade com configurações de versões anteriores.
                    hsv_default = {
                        "h_min": 5, "s_min": 100, "v_min": 100,
                        "h_max": 22, "s_max": 255, "v_max": 255
                    }
                    hsv_default.update(data.get("hsv", {}))
                    defaults["hsv"] = hsv_default
                    defaults.setdefault("min_area", 500)
                    defaults.setdefault("max_area", 100000)
                    defaults.setdefault("hsv_profiles", {})
                    vt={"stable_frames":8,"max_match_px":55,"max_missed_frames":30,"stable_move_px":7,"recovery_match_px":90,"area_tolerance_pct":45}
                    vt.update(data.get("vision_tracking",{}))
                    defaults["vision_tracking"]=vt
                    defaults.setdefault("tool_offset_z", 0.0)
                    defaults.setdefault("z_min_limit", -800.0)
                    h={"x":0.0,"y":0.0,"z":-550.0,"tool":"OFF"}
                    h.update(data.get("holding",{}))
                    defaults["holding"]=h
                    pp={"approach_z":-550.0,"pick_z":-650.0,"safe_z":-500.0,"drop_x":150.0,"drop_y":0.0,"drop_z":-650.0}
                    pp.update(data.get("pick_place",{}))
                    defaults["pick_place"]=pp
            except Exception as e:
                print(f"Erro ao carregar configurações: {e}")
        return defaults

    @staticmethod
    def save(data):
        try:
            with open(CONFIG_FILE, "w") as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            print(f"Erro ao salvar configurações: {e}")

# ==============================================================================
# WORKER SERIAL (PROTOCOLO M3.9)
# ==============================================================================
class DeltaSerialWorker(QThread):
    sig_state_changed = pyqtSignal(str)
    sig_log = pyqtSignal(str)
    sig_command_done = pyqtSignal(int, str)
    sig_error_received = pyqtSignal(int, str)
    sig_boot_ready = pyqtSignal()
    sig_reboot_detected = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = True
        self.cmd_counter = 1
        self.current_state = "DESCONECTADO"
        self.boot_sync_pending = False
        self._serial_lock = threading.RLock()

    def connect_serial(self, port, baud=115200):
        with self._serial_lock:
            previous = self.ser
            self.ser = None
            self._close_port(previous)
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.boot_sync_pending = True
                self.current_state = "CONECTADO"
                self.sig_state_changed.emit(self.current_state)
                self.sig_log.emit(f"[HARDWARE] Conexão ativa em {port} @ {baud} bps")
                return True
            except Exception as e:
                self.current_state = "DESCONECTADO"
                self.sig_state_changed.emit(self.current_state)
                self.sig_log.emit(f"[ERRO SERIAL] {e}")
                return False

    def disconnect_serial(self):
        with self._serial_lock:
            ser = self.ser
            self.ser = None
            self._close_port(ser)
            self.current_state = "DESCONECTADO"
            self.sig_state_changed.emit(self.current_state)
            self.sig_log.emit("[HARDWARE] Comunicação encerrada.")

    def _close_port(self, ser):
        if ser:
            try:
                if ser.is_open:
                    ser.close()
            except (serial.SerialException, OSError) as e:
                self.sig_log.emit(f"[FALHA SERIAL] Erro ao fechar a porta: {e}")

    def _handle_serial_failure(self, ser, context, error):
        if self.ser is ser:
            self.ser = None
        self._close_port(ser)
        self.current_state = "DESCONECTADO"
        self.sig_state_changed.emit(self.current_state)
        self.sig_log.emit(f"[{context}] Comunicação serial encerrada: {error}")

    def send_cmd(self, command, params=""):
        with self._serial_lock:
            ser = self.ser
            if not ser or not ser.is_open:
                self.sig_log.emit("[ALERTA] Porta Serial não conectada.")
                return -1

            cmd_id = self.cmd_counter
            frame = f"CMD|{cmd_id:04d}|{command}|{params}\n" if params else f"CMD|{cmd_id:04d}|{command}\n"
            try:
                ser.write(frame.encode('ascii'))
            except (serial.SerialException, OSError) as e:
                self._handle_serial_failure(ser, "FALHA SERIAL TX", e)
                return -1
            self.cmd_counter = (self.cmd_counter % 9999) + 1

        self.sig_log.emit(f"TX -> {frame.strip()}")
        return cmd_id

    def run(self):
        rx_buf = ""
        while self.running:
            ser = None
            try:
                with self._serial_lock:
                    ser = self.ser
                    if ser and ser.is_open:
                        while ser.in_waiting > 0:
                            char = ser.read().decode('ascii', errors='ignore')
                            if char == '\r':
                                continue
                            if char == '\n':
                                self.parse_frame(rx_buf.strip())
                                rx_buf = ""
                            else:
                                rx_buf += char
            except (serial.SerialException, OSError) as e:
                if ser is not None:
                    with self._serial_lock:
                        self._handle_serial_failure(ser, "FALHA SERIAL RX", e)
                else:
                    self.sig_log.emit(f"[FALHA SERIAL RX] {e}")
            self.msleep(10)

    def parse_frame(self, line):
        if not line:
            return
        self.sig_log.emit(f"RX <- {line}")

        # Reboot/rearme físico detectado com a serial ainda aberta.
        # Rearma o sincronismo para que, após o banner "Estado:", a GUI
        # consulte STATUS novamente sem exigir desconectar/reconectar a COM.
        if line.startswith("RESET_BUTTON|FULL_RESTART") or line.startswith("rst:"):
            self.boot_sync_pending = True
            self.sig_reboot_detected.emit()

        # Sincronização de boot: ao abrir a USB o ESP32 reinicia.
        # A primeira consulta STATUS só é liberada após o próprio firmware
        # anunciar seu estado inicial, evitando comandos durante o bootloader.
        if line.startswith("Estado:"):
            boot_state = line.split(":", 1)[1].strip()
            if boot_state:
                self.current_state = boot_state
                self.sig_state_changed.emit(boot_state)
            if self.boot_sync_pending:
                self.boot_sync_pending = False
                self.sig_boot_ready.emit()
            return

        parts = line.split('|')

        # Eventos assíncronos da emergência física — M3.9.
        if parts[0] == "EMERGENCY":
            event = parts[1] if len(parts) > 1 else ""
            if event == "ACTIVE":
                self.current_state = "EMERGENCY"
                self.sig_state_changed.emit("EMERGENCY")
            elif event == "RELEASED":
                # Liberação física não rearma o robô.
                self.current_state = "EMERGENCY_RELEASED"
                self.sig_state_changed.emit("EMERGENCY_RELEASED")
            return

        if parts[0] == "STATUS":
            st = parts[2] if (len(parts) >= 3 and parts[1].isdigit()) else parts[1]
            self.current_state = st
            self.sig_state_changed.emit(st)

        elif parts[0] == "OK":
            if len(parts) >= 3:
                cid = int(parts[1]) if parts[1].isdigit() else 0
                cmd = parts[2]
                self.sig_command_done.emit(cid, cmd)

        elif parts[0] == "ERROR":
            cid = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
            if len(parts) > 2:
                err = "|".join(parts[2:])
            elif len(parts) > 1:
                err = parts[1]
            else:
                err = "UNKNOWN"
            self.sig_error_received.emit(cid, err)

    def stop(self):
        self.running = False
        self.disconnect_serial()
        self.wait()

# ==============================================================================
# FILTRO DE KALMAN 2D PARA COORDENADAS (X, Y)
# ==============================================================================
class Kalman2D:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                              [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                             [0, 1, 0, 1],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.initialized = False

    def update(self, x, y):
        if not self.initialized:
            self.kf.statePre = np.array([[x], [y], [0], [0]], np.float32)
            self.kf.statePost = np.array([[x], [y], [0], [0]], np.float32)
            self.initialized = True
        self.kf.predict()
        medicao = np.array([[x], [y]], np.float32)
        estimado = self.kf.correct(medicao)
        return float(estimado[0][0]), float(estimado[1][0])

    def reset(self):
        self.initialized = False

# ==============================================================================
# WORKER DE VISÃO COMPUTACIONAL ROBUSTO
# ==============================================================================
class CameraWorker(QThread):
    sig_frame_processed = pyqtSignal(QImage, QImage)
    sig_calib_frame = pyqtSignal(QImage)
    sig_target_detected = pyqtSignal(float, float, str, float)
    sig_objects = pyqtSignal(object)

    def __init__(self, hsv_initial, initial_index=0):
        super().__init__()
        self.running = True
        self.cap = None
        self.camera_index = initial_index
        self.hsv_bounds = hsv_initial
        self.min_area = 500
        self.max_area = 100000
        self.tracks = {}
        self.next_track_id = 1
        self.stable_frames_required = 8
        self.max_match_px = 55.0
        self.max_missed_frames = 30
        self.stable_move_px = 7.0
        self.recovery_match_px = 90.0
        self.area_tolerance_pct = 45.0
        self.H = None
        self.kalman = Kalman2D()
        self.load_homography()
        self.calib_points = []
        self.target_lost_counter = 0

    def set_camera_index(self, index):
        if self.cap and self.cap.isOpened():
            self.cap.release()
            time.sleep(0.3)
        self.camera_index = index
        self.init_camera()

    def load_homography(self):
        if os.path.exists(HOMOGRAPHY_FILE):
            try:
                self.H = np.load(HOMOGRAPHY_FILE)
            except:
                self.H = None
        else:
            self.H = None

    def update_hsv(self, bounds):
        self.hsv_bounds = dict(bounds)

    def update_area_limits(self, min_area, max_area):
        self.min_area = int(min_area)
        self.max_area = int(max_area)

    def update_tracking_config(self, cfg):
        self.stable_frames_required=max(2,int(cfg.get("stable_frames",8)))
        self.max_match_px=max(5.0,float(cfg.get("max_match_px",55)))
        self.max_missed_frames=max(1,int(cfg.get("max_missed_frames",30)))
        self.stable_move_px=max(1.0,float(cfg.get("stable_move_px",7)))
        self.recovery_match_px=max(self.max_match_px,float(cfg.get("recovery_match_px",90)))
        self.area_tolerance_pct=max(5.0,min(100.0,float(cfg.get("area_tolerance_pct",45))))

    def reset_tracks(self):
        self.tracks={}; self.next_track_id=1

    def update_tracks(self,detections):
        unmatched_t=set(self.tracks)
        unmatched_d=set(range(len(detections)))
        candidates=[]

        # Associação normal: centróide próximo + coerência básica de forma/área.
        for tid,tr in self.tracks.items():
            for di,d in enumerate(detections):
                dist=((d["cx"]-tr["cx"])**2+(d["cy"]-tr["cy"])**2)**0.5
                limit=self.recovery_match_px if tr.get("state")=="LOST" else self.max_match_px

                old_area=max(1.0,float(tr.get("area",1.0)))
                area_diff=abs(float(d["area"])-old_area)/old_area*100.0
                shape_ok=(tr.get("shape")==d.get("shape") or
                          tr.get("shape")=="Indefinido" or d.get("shape")=="Indefinido")

                if dist<=limit and area_diff<=self.area_tolerance_pct and shape_ok:
                    # LOST recebe pequena penalização para preferir tracks visíveis próximos.
                    score=dist + (12.0 if tr.get("state")=="LOST" else 0.0) + area_diff*0.15
                    candidates.append((score,tid,di))

        for _,tid,di in sorted(candidates):
            if tid not in unmatched_t or di not in unmatched_d:
                continue

            tr=self.tracks[tid]
            d=detections[di]
            was_lost=(tr.get("state")=="LOST")
            move=((d["cx"]-tr["cx"])**2+(d["cy"]-tr["cy"])**2)**0.5

            tr.update(d)
            tr["missed"]=0

            # Depois de uma oclusão o objeto precisa reconquistar estabilidade.
            if was_lost:
                tr["stable_count"]=1
                tr["state"]="DETECTED"
            else:
                tr["stable_count"]=tr.get("stable_count",1)+1 if move<=self.stable_move_px else 1
                tr["state"]="STABLE" if tr["stable_count"]>=self.stable_frames_required else "DETECTED"

            unmatched_t.remove(tid)
            unmatched_d.remove(di)

        # Track não visto permanece em memória como LOST durante a janela configurada.
        for tid in list(unmatched_t):
            tr=self.tracks[tid]
            tr["missed"]=tr.get("missed",0)+1
            tr["stable_count"]=0
            tr["state"]="LOST"
            if tr["missed"]>self.max_missed_frames:
                del self.tracks[tid]

        # Só cria novo ID quando nenhuma identidade existente é compatível.
        for di in unmatched_d:
            tid=self.next_track_id
            self.next_track_id+=1
            self.tracks[tid]=dict(detections[di],id=tid,stable_count=1,missed=0,state="DETECTED")

        # Inclui LOST na tabela para tornar a memória do tracker observável.
        return [dict(tr,id=tid) for tid,tr in sorted(self.tracks.items())]

    def set_calib_points(self, pts):
        self.calib_points = pts

    def init_camera(self):
        # Utiliza CAP_DSHOW no Windows para inicialização rápida e estável
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def run(self):
        self.init_camera()
        while self.running:
            if not self.cap or not self.cap.isOpened():
                self.msleep(150)
                continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.msleep(30)
                continue

            # Frame da aba de calibração
            frame_calib = frame.copy()
            for i, p in enumerate(self.calib_points):
                cv2.circle(frame_calib, (p[0], p[1]), 6, (0, 0, 255), -1)
                cv2.putText(frame_calib, f"P{i+1}", (p[0]+10, p[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if len(self.calib_points) == 4:
                pts_poly = np.array(self.calib_points, np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame_calib, [pts_poly], isClosed=True, color=(0, 255, 0), thickness=2)

            q_calib = self.cv2_to_qimage(frame_calib)
            self.sig_calib_frame.emit(q_calib)

            # Pré-processamento e filtragem HSV
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            lower = np.array([self.hsv_bounds['h_min'], self.hsv_bounds['s_min'], self.hsv_bounds['v_min']])
            upper = np.array([self.hsv_bounds['h_max'], self.hsv_bounds['s_max'], self.hsv_bounds['v_max']])
            mask = cv2.inRange(hsv, lower, upper)

            # PC V1: quando os 4 pontos estiverem definidos, a visão considera somente a ROI calibrada.
            if len(self.calib_points) == 4:
                roi_mask = np.zeros(mask.shape, dtype=np.uint8)
                pts_roi = np.array(self.calib_points, dtype=np.int32).reshape((-1, 1, 2))
                cv2.fillPoly(roi_mask, [pts_roi], 255)
                mask = cv2.bitwise_and(mask, roi_mask)
                cv2.polylines(frame, [pts_roi], True, (255, 120, 0), 2)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detections=[]
            for cnt in contours:
                area=cv2.contourArea(cnt)
                if not (self.min_area <= area <= self.max_area): continue
                peri=cv2.arcLength(cnt,True); approx=cv2.approxPolyDP(cnt,0.04*peri,True); vertices=len(approx)
                formato="Indefinido"
                if vertices==3: formato="Triangulo"
                elif vertices==4:
                    _,_,w,h=cv2.boundingRect(approx)
                    formato="Quadrado" if h and 0.85 <= w/float(h) <= 1.15 else "Retangulo"
                elif vertices>4: formato="Circulo"
                M=cv2.moments(cnt)
                if M["m00"]==0: continue
                cx=int(M["m10"]/M["m00"]); cy=int(M["m01"]/M["m00"])
                angle=float(cv2.minAreaRect(cnt)[2]); rx=ry=None
                if self.H is not None:
                    pt=np.array([[[cx,cy]]],dtype=np.float32)
                    rp=cv2.perspectiveTransform(pt,self.H)[0][0]
                    rx=float(rp[0]); ry=float(rp[1])
                detections.append({"cx":cx,"cy":cy,"x":rx,"y":ry,"shape":formato,"angle":angle,"area":float(area)})

            objects=self.update_tracks(detections)
            stable_objects=[]
            for o in objects:
                if o["state"]=="LOST":
                    continue
                stable=o["state"]=="STABLE"
                color=(0,255,0) if stable else (0,200,255)
                cv2.circle(frame,(o["cx"],o["cy"]),5,color,-1)
                cv2.putText(frame,f'ID {o["id"]} {o["state"]}',(o["cx"]+8,o["cy"]-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.46,color,1,cv2.LINE_AA)
                if o["x"] is not None:
                    cv2.putText(frame,f'{o["shape"]} X:{o["x"]:.1f} Y:{o["y"]:.1f}',(o["cx"]+8,o["cy"]+10),
                                cv2.FONT_HERSHEY_SIMPLEX,0.42,color,1,cv2.LINE_AA)
                if stable and o["x"] is not None: stable_objects.append(o)
            self.sig_objects.emit(objects)

            # Compatibilidade com a exibição antiga: primeiro objeto estável é mostrado
            # como alvo visual, mas não existe disparo automático de movimento.
            if stable_objects:
                o=stable_objects[0]
                self.sig_target_detected.emit(o["x"],o["y"],o["shape"],o["angle"])
                self.target_lost_counter=0
            else:
                self.target_lost_counter+=1
                if self.target_lost_counter>10: self.kalman.reset()

            q_frame = self.cv2_to_qimage(frame)
            q_mask = QImage(mask.data, mask.shape[1], mask.shape[0], mask.shape[1], QImage.Format_Grayscale8)
            self.sig_frame_processed.emit(q_frame, q_mask)

            self.msleep(30)

        if self.cap:
            self.cap.release()

    def cv2_to_qimage(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        return QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()

    def stop(self):
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.wait()

# ==============================================================================
# WIDGET DE VÍDEO CLICÁVEL (CALIBRAÇÃO)
# ==============================================================================
class ClickableVideoLabel(QLabel):
    sig_clicked_coords = pyqtSignal(int, int)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.sig_clicked_coords.emit(event.pos().x(), event.pos().y())
        super().mousePressEvent(event)

# ==============================================================================
# JANELA PRINCIPAL DA IDE (V3.1)
# ==============================================================================


V1_QSS = """
QMainWindow, QWidget { background:#151a22; color:#eef2f7; font-family:'Segoe UI'; font-size:10pt; }
QFrame#topbar, QFrame#sidebar, QFrame#statusbar { background:#1c2430; border:1px solid #2d3746; border-radius:10px; }
QLabel#brand { color:#3f6fff; font-size:18pt; font-weight:700; }
QLabel#subtitle { color:#9aa7b7; }
QLabel#state { font-size:16pt; font-weight:700; padding:8px 14px; border-radius:8px; }
QLabel#cardTitle { color:#9fb7ff; font-size:11pt; font-weight:700; }
QGroupBox { border:1px solid #303b4a; border-radius:9px; margin-top:12px; padding-top:12px; font-weight:600; color:#a8bcff; }
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 6px; }
QPushButton { background:#315fe9; color:white; border:0; border-radius:7px; padding:9px 13px; font-weight:600; }
QPushButton:hover { background:#416fff; }
QPushButton:disabled { background:#343b47; color:#737e8d; }
QPushButton#nav { text-align:left; background:transparent; color:#b9c3d0; padding:11px 14px; }
QPushButton#nav:checked { background:#294fc4; color:white; }
QPushButton#start { background:#258a55; }
QPushButton#pause { background:#d39b20; color:#10151c; }
QPushButton#resume { background:#258a55; }
QPushButton#stop { background:#c83b3b; font-size:11pt; padding:11px 18px; }
QPushButton#reset { background:#315fe9; }
QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox { background:#202936; border:1px solid #3a4658; border-radius:6px; padding:7px; color:#eef2f7; }
QTextEdit { background:#0f141b; border:1px solid #2e3948; border-radius:7px; color:#b7f5c9; font-family:Consolas; }
QListWidget { background:#171e28; border:0; outline:0; }
QSlider::groove:horizontal { height:6px; background:#303b4a; border-radius:3px; }
QSlider::sub-page:horizontal { background:#416fff; border-radius:3px; }
QSlider::handle:horizontal { background:#e7ecf3; width:14px; margin:-4px 0; border-radius:7px; }
"""

class DeltaRobotIDE(QMainWindow):
    """DELTA VISION CONTROL — PC V1.1.0 / M3.9 + PICK PLACE SEMIASSISTIDO."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DELTA VISION CONTROL — PC V1.1.0 | ESP32 #2 M3.9 | PICK & PLACE SEMIASSISTIDO")
        self.resize(1360, 820)
        self.setMinimumSize(1080, 700)
        self.cfg = ConfigManager.load()
        self.setStyleSheet(V1_QSS)
        self.serial_worker = DeltaSerialWorker()
        self.camera_worker = CameraWorker(self.cfg["hsv"], self.cfg["camera_index"])
        self.camera_worker.update_area_limits(self.cfg.get("min_area", 500), self.cfg.get("max_area", 100000))
        self.camera_worker.update_tracking_config(self.cfg.get("vision_tracking", {}))
        self.target_x = self.target_y = self.target_angle = 0.0
        self.target_shape = "Nenhum"
        self.calib_clicks_px = []
        self.last_calib_qimage = None
        self.last_cmd_id = None
        self.initial_status_sent = False
        # Referência cartesiana do TEACH. É atualizada somente após MOVEJ concluído com OK.
        self.teach_x = 0.0
        self.teach_y = 0.0
        self.teach_z = -600.0
        self.teach_tool = "OFF"
        self.teach_pending = None
        self.tool_offset_z = abs(float(self.cfg.get("tool_offset_z", 0.0)))
        self.z_min_limit = -abs(float(self.cfg.get("z_min_limit", -800.0)))
        self.holding = dict(self.cfg.get("holding", {"x":0.0,"y":0.0,"z":-550.0,"tool":"OFF"}))
        self.last_objects = []
        self.pick_place = dict(self.cfg.get("pick_place", {}))
        self.pick_target = None
        self.pick_cycle_steps = []
        self.pick_cycle_index = 0
        self.pick_cycle_waiting_id = None
        self.pick_cycle_active = False
        self.build_ui()
        self.connect_signals()
        self.serial_worker.start(); self.camera_worker.start()
        self.refresh_ports(); self.populate_camera_list()
        self.on_robot_state_changed("DESCONECTADO")
        self.show_page(0)

    def build_ui(self):
        root=QWidget(); self.setCentralWidget(root)
        outer=QVBoxLayout(root); outer.setContentsMargins(10,10,10,10); outer.setSpacing(8)
        top=QFrame(); top.setObjectName('topbar'); tl=QHBoxLayout(top)
        self.lbl_logo=QLabel(); self.lbl_logo.setFixedSize(120,48); self.lbl_logo.setAlignment(Qt.AlignCenter)
        if os.path.exists(LOGO_FILE): self.lbl_logo.setPixmap(QPixmap(LOGO_FILE).scaled(self.lbl_logo.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
        else: self.lbl_logo.setText('UNIVERSIDADE')
        tl.addWidget(self.lbl_logo)
        titles=QVBoxLayout(); brand=QLabel('DELTA VISION CONTROL'); brand.setObjectName('brand'); sub=QLabel('Supervisão Robótica • Visão Computacional • ESP32 #2 M3.9'); sub.setObjectName('subtitle'); titles.addWidget(brand); titles.addWidget(sub); tl.addLayout(titles); tl.addStretch()
        self.lbl_serial_dot=QLabel('● ESP32 OFFLINE'); self.lbl_camera_dot=QLabel('● CÂMERA'); self.lbl_calib_dot=QLabel('● CALIBRAÇÃO')
        tl.addWidget(self.lbl_serial_dot); tl.addWidget(self.lbl_camera_dot); tl.addWidget(self.lbl_calib_dot)
        outer.addWidget(top)

        body=QHBoxLayout(); body.setSpacing(8)
        side=QFrame(); side.setObjectName('sidebar'); side.setFixedWidth(180); sl=QVBoxLayout(side)
        self.nav_buttons=[]
        pages=[('⌂  HOME',0),('◉  VISÃO',1),('◎  CALIBRAÇÃO',2),('🎨  CORES',3),('⚙  REGRAS',4),('⌖  DESTINOS',5),('☷  FILA',6),('🎮  TEACH / JOG',7),('▥  MÉTRICAS',8),('?  MANUAL',9),('⌘  DIAGNÓSTICO',10)]
        for text,idx in pages:
            b=QPushButton(text); b.setObjectName('nav'); b.setCheckable(True); b.clicked.connect(lambda _,i=idx:self.show_page(i)); sl.addWidget(b); self.nav_buttons.append(b)
        sl.addStretch(); body.addWidget(side)
        self.stack=QStackedWidget(); body.addWidget(self.stack,1)
        self.build_home(); self.build_vision(); self.build_calibration(); self.build_colors(); self.build_placeholder('REGRAS','Classificação Cor + Forma → Destino será habilitada em uma etapa posterior.'); self.build_placeholder('DESTINOS','Destinos e zonas DROP / IGNORE / SAFETY serão configurados aqui.'); self.build_tracking(); self.build_control(); self.build_placeholder('MÉTRICAS','Ciclos, tempos, taxa de sucesso e indicadores experimentais serão registrados aqui.'); self.build_manual(); self.build_diagnostics()
        outer.addLayout(body,1)

        bar=QFrame(); bar.setObjectName('statusbar'); bl=QHBoxLayout(bar)
        self.lbl_robot_state=QLabel('DESCONECTADO'); self.lbl_robot_state.setObjectName('state'); bl.addWidget(self.lbl_robot_state)
        self.lbl_guidance=QLabel('Conecte o ESP32 #2 para iniciar.'); self.lbl_guidance.setObjectName('subtitle'); bl.addWidget(self.lbl_guidance,1)
        self.btn_pause=QPushButton('Ⅱ  PAUSE'); self.btn_pause.setObjectName('pause')
        self.btn_resume=QPushButton('▶  RESUME'); self.btn_resume.setObjectName('resume')
        self.btn_stop=QPushButton('■  STOP'); self.btn_stop.setObjectName('stop')
        bl.addWidget(self.btn_pause); bl.addWidget(self.btn_resume); bl.addWidget(self.btn_stop); outer.addWidget(bar)

    def page_shell(self,title,subtitle=''):
        w=QWidget(); l=QVBoxLayout(w); h=QLabel(title); h.setObjectName('brand'); l.addWidget(h)
        if subtitle: s=QLabel(subtitle); s.setObjectName('subtitle'); s.setWordWrap(True); l.addWidget(s)
        self.stack.addWidget(w); return w,l

    def build_home(self):
        w,l=self.page_shell('HOME','Visão geral da plataforma e sequência segura de inicialização.')
        hero=QHBoxLayout(); image=QLabel('IMAGEM DO ROBÔ\n\nAdicione robot.png na pasta do programa'); image.setAlignment(Qt.AlignCenter); image.setMinimumSize(520,360); image.setStyleSheet('background:#0e131a;border:1px dashed #445166;border-radius:12px;color:#778397;')
        if os.path.exists('robot.png'): image.setPixmap(QPixmap('robot.png').scaled(620,420,Qt.KeepAspectRatio,Qt.SmoothTransformation))
        hero.addWidget(image,2)
        right=QVBoxLayout(); title=QLabel('PLATAFORMA DELTA'); title.setObjectName('brand'); right.addWidget(title)
        self.lbl_home_state=QLabel('Aguardando controlador...'); self.lbl_home_state.setWordWrap(True); right.addWidget(self.lbl_home_state)
        checks=QLabel('INICIALIZAÇÃO\n\n1. Conectar ESP32 #2\n2. Confirmar NOT_HOMED\n3. Executar HOME\n4. Aguardar READY\n5. Validar comandos M3.9'); checks.setStyleSheet('background:#1c2430;border-radius:10px;padding:18px;line-height:150%;'); right.addWidget(checks)
        self.btn_go_control=QPushButton('IR PARA CONTROLE'); right.addWidget(self.btn_go_control); right.addStretch(); hero.addLayout(right,1); l.addLayout(hero,1)

    def build_vision(self):
        w,l=self.page_shell('VISÃO','Monitoramento visual. Visão em monitoramento. O MVP automático permanece bloqueado enquanto validamos o TEACH/JOG.')
        row=QHBoxLayout(); self.lbl_cam_main=QLabel('Aguardando câmera...'); self.lbl_cam_main.setAlignment(Qt.AlignCenter); self.lbl_cam_main.setMinimumSize(640,480); self.lbl_cam_main.setStyleSheet('background:#000;border-radius:10px;'); row.addWidget(self.lbl_cam_main,3)
        side=QVBoxLayout(); grp=QGroupBox('Câmera'); gl=QVBoxLayout(grp); self.cb_cameras=QComboBox(); gl.addWidget(self.cb_cameras); side.addWidget(grp)
        self.lbl_target_info=QLabel('TARGET: não validado'); self.lbl_target_info.setWordWrap(True); self.lbl_target_info.setStyleSheet('background:#1c2430;border-radius:8px;padding:12px;'); side.addWidget(self.lbl_target_info)
        note=QLabel('INTERTRAVAMENTO V1\n\nA detecção é apenas informativa. O ciclo automático permanece bloqueado até concluirmos a validação serial M3.9.'); note.setWordWrap(True); note.setStyleSheet('color:#d6b35d;'); side.addWidget(note); side.addStretch(); row.addLayout(side,1); l.addLayout(row,1)

    def build_calibration(self):
        w,l=self.page_shell('CALIBRAÇÃO','Defina os quatro vértices da área operacional. A detecção será mascarada para dentro desta ROI.')
        row=QHBoxLayout(); self.lbl_cam_calib=ClickableVideoLabel(); self.lbl_cam_calib.setAlignment(Qt.AlignCenter); self.lbl_cam_calib.setMinimumSize(640,480); self.lbl_cam_calib.setStyleSheet('background:#000;border:1px solid #416fff;border-radius:10px;'); row.addWidget(self.lbl_cam_calib,2)
        panel=QVBoxLayout(); grp=QGroupBox('Coordenadas reais dos vértices'); grid=QGridLayout(grp); self.spn_coords=[]
        defaults=[(-300,320),(300,320),(300,-200),(-300,-200)]
        for i,(dx,dy) in enumerate(defaults):
            sx=QDoubleSpinBox(); sy=QDoubleSpinBox(); sx.setRange(-500,500); sy.setRange(-500,500); sx.setValue(dx); sy.setValue(dy); self.spn_coords.append((sx,sy)); grid.addWidget(QLabel(f'P{i+1} X'),i,0); grid.addWidget(sx,i,1); grid.addWidget(QLabel('Y'),i,2); grid.addWidget(sy,i,3)
        panel.addWidget(grp); self.btn_reset_calib=QPushButton('LIMPAR PONTOS'); self.btn_save_homography=QPushButton('CALCULAR & SALVAR HOMOGRAFIA'); panel.addWidget(self.btn_reset_calib); panel.addWidget(self.btn_save_homography); panel.addStretch(); row.addLayout(panel,1); l.addLayout(row,1)

    def build_tracking(self):
        w,l=self.page_shell('FILA / TRACKING','Multiobjetos + estabilidade + recuperação de ID após oclusão curta. Movimento automático permanece bloqueado.')
        ctl=QHBoxLayout(); vt=self.cfg.get('vision_tracking',{})
        self.sp_stable_frames=QSpinBox(); self.sp_stable_frames.setRange(2,60); self.sp_stable_frames.setValue(int(vt.get('stable_frames',8)))
        self.sp_match_px=QSpinBox(); self.sp_match_px.setRange(5,200); self.sp_match_px.setValue(int(vt.get('max_match_px',55)))
        self.sp_stable_px=QSpinBox(); self.sp_stable_px.setRange(1,50); self.sp_stable_px.setValue(int(vt.get('stable_move_px',7)))
        self.sp_missed=QSpinBox(); self.sp_missed.setRange(1,180); self.sp_missed.setValue(int(vt.get('max_missed_frames',30)))
        self.sp_recovery_px=QSpinBox(); self.sp_recovery_px.setRange(10,300); self.sp_recovery_px.setValue(int(vt.get('recovery_match_px',90)))
        self.sp_area_tol=QSpinBox(); self.sp_area_tol.setRange(5,100); self.sp_area_tol.setValue(int(vt.get('area_tolerance_pct',45)))
        for t,wid in [('Frames STABLE',self.sp_stable_frames),('Match px',self.sp_match_px),('Estável px',self.sp_stable_px),
                      ('Janela LOST',self.sp_missed),('Recuperação px',self.sp_recovery_px),('Área ±%',self.sp_area_tol)]:
            ctl.addWidget(QLabel(t)); ctl.addWidget(wid)
        self.btn_reset_tracks=QPushButton('RESETAR IDs'); ctl.addWidget(self.btn_reset_tracks); ctl.addStretch(); l.addLayout(ctl)
        self.tbl_objects=QTableWidget(0,8)
        self.tbl_objects.setHorizontalHeaderLabels(['ID','ESTADO','X mm','Y mm','PIXEL X','PIXEL Y','FORMA','ÁREA px²'])
        self.tbl_objects.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl_objects.setEditTriggers(QAbstractItemView.NoEditTriggers)
        l.addWidget(self.tbl_objects,1)
        info=QLabel('DETECTED = aguardando estabilidade. STABLE = objeto praticamente imóvel durante N frames. '
                    'LOST = objeto temporariamente oculto, mantido em memória para recuperar o MESMO ID. '
                    'LOST nunca é alvo válido. Após a janela LOST, o track é removido.')
        info.setWordWrap(True); info.setStyleSheet('color:#a8bcff;padding:8px;'); l.addWidget(info)

    def apply_tracking_controls(self):
        if not hasattr(self,'sp_stable_frames'): return
        cfg={'stable_frames':self.sp_stable_frames.value(),'max_match_px':self.sp_match_px.value(),
             'stable_move_px':self.sp_stable_px.value(),'max_missed_frames':self.sp_missed.value(),
             'recovery_match_px':self.sp_recovery_px.value(),'area_tolerance_pct':self.sp_area_tol.value()}
        self.cfg['vision_tracking']=cfg; self.camera_worker.update_tracking_config(cfg)

    def update_objects_table(self,objects):
        self.last_objects=list(objects)
        if hasattr(self,'lbl_teach_objects'):
            if not objects:
                txt='OBJETOS: nenhum objeto detectado'
            else:
                lines=['OBJETOS VISÍVEIS / MEMÓRIA DO TRACKER']
                for o in objects[:6]:
                    xy='X --  Y --' if o.get('x') is None else f"X {o['x']:.1f}  Y {o['y']:.1f}"
                    lines.append(f"ID {o.get('id','?')} | {o.get('state','')} | {xy} | {o.get('shape','')}")
                txt='\n'.join(lines)
            self.lbl_teach_objects.setText(txt)
        if not hasattr(self,'tbl_objects'): return
        self.tbl_objects.setRowCount(len(objects))
        for r,o in enumerate(objects):
            vals=[o.get('id',''),o.get('state',''),
                  '' if o.get('x') is None else f"{o['x']:.1f}",
                  '' if o.get('y') is None else f"{o['y']:.1f}",
                  o.get('cx',''),o.get('cy',''),o.get('shape',''),f"{o.get('area',0):.0f}"]
            for c,v in enumerate(vals): self.tbl_objects.setItem(r,c,QTableWidgetItem(str(v)))

    def build_colors(self):
        w,l=self.page_shell('CORES / HSV','Ajuste a segmentação em tempo real antes dos testes métricos da homografia.')
        row=QHBoxLayout()

        preview=QVBoxLayout()
        self.lbl_hsv_mask=QLabel('MÁSCARA HSV\nAguardando câmera...')
        self.lbl_hsv_mask.setAlignment(Qt.AlignCenter)
        self.lbl_hsv_mask.setMinimumSize(640,480)
        self.lbl_hsv_mask.setStyleSheet('background:#000;border:1px solid #416fff;border-radius:10px;')
        preview.addWidget(self.lbl_hsv_mask,1)
        self.lbl_hsv_summary=QLabel()
        self.lbl_hsv_summary.setWordWrap(True)
        self.lbl_hsv_summary.setStyleSheet('background:#1c2430;border-radius:8px;padding:10px;color:#a8bcff;')
        preview.addWidget(self.lbl_hsv_summary)
        row.addLayout(preview,2)

        panel=QVBoxLayout()
        grp=QGroupBox('Limites HSV — OpenCV')
        grid=QGridLayout(grp)
        self.hsv_sliders={}
        self.hsv_value_labels={}

        specs=[
            ('h_min','H mínimo',0,179), ('h_max','H máximo',0,179),
            ('s_min','S mínimo',0,255), ('s_max','S máximo',0,255),
            ('v_min','V mínimo',0,255), ('v_max','V máximo',0,255)
        ]
        current=self.cfg.get('hsv',{})
        for r,(key,title,vmin,vmax) in enumerate(specs):
            lab=QLabel(title)
            slider=QSlider(Qt.Horizontal); slider.setRange(vmin,vmax); slider.setValue(int(current.get(key,vmin)))
            value=QLabel(str(slider.value())); value.setFixedWidth(38); value.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
            self.hsv_sliders[key]=slider; self.hsv_value_labels[key]=value
            grid.addWidget(lab,r,0); grid.addWidget(slider,r,1); grid.addWidget(value,r,2)
        panel.addWidget(grp)

        area_grp=QGroupBox('Filtro por área do contorno')
        ag=QGridLayout(area_grp)
        self.sp_min_area=QSpinBox(); self.sp_min_area.setRange(1,500000); self.sp_min_area.setValue(int(self.cfg.get('min_area',500)))
        self.sp_max_area=QSpinBox(); self.sp_max_area.setRange(1,500000); self.sp_max_area.setValue(int(self.cfg.get('max_area',100000)))
        ag.addWidget(QLabel('Área mínima (px²)'),0,0); ag.addWidget(self.sp_min_area,0,1)
        ag.addWidget(QLabel('Área máxima (px²)'),1,0); ag.addWidget(self.sp_max_area,1,1)
        panel.addWidget(area_grp)

        prof=QGroupBox('Perfis de cor')
        pg=QGridLayout(prof)
        self.cb_hsv_profiles=QComboBox()
        self.ed_hsv_profile=QLineEdit(); self.ed_hsv_profile.setPlaceholderText('Ex.: LARANJA')
        self.btn_hsv_save=QPushButton('SALVAR PERFIL')
        self.btn_hsv_load=QPushButton('CARREGAR PERFIL')
        self.btn_hsv_delete=QPushButton('EXCLUIR PERFIL')
        pg.addWidget(QLabel('Perfis salvos'),0,0); pg.addWidget(self.cb_hsv_profiles,0,1,1,2)
        pg.addWidget(QLabel('Nome'),1,0); pg.addWidget(self.ed_hsv_profile,1,1,1,2)
        pg.addWidget(self.btn_hsv_save,2,0); pg.addWidget(self.btn_hsv_load,2,1); pg.addWidget(self.btn_hsv_delete,2,2)
        panel.addWidget(prof)

        note=QLabel('AJUSTE RECOMENDADO\nDeixe o objeto branco na máscara e o fundo preto. '
                    'A mão e reflexos devem ser eliminados sempre que possível. '
                    'As alterações HSV e de área são aplicadas à câmera em tempo real.')
        note.setWordWrap(True); note.setStyleSheet('color:#d6b35d;padding:8px;')
        panel.addWidget(note); panel.addStretch()
        row.addLayout(panel,1); l.addLayout(row,1)
        self.refresh_hsv_profiles()
        self.apply_hsv_controls()

    def current_hsv_from_controls(self):
        return {k:int(sl.value()) for k,sl in self.hsv_sliders.items()}

    def apply_hsv_controls(self):
        if not hasattr(self,'hsv_sliders'): return
        hsv=self.current_hsv_from_controls()
        # Garante intervalos válidos sem alterar silenciosamente os sliders.
        valid = hsv['h_min'] <= hsv['h_max'] and hsv['s_min'] <= hsv['s_max'] and hsv['v_min'] <= hsv['v_max']
        for k,lab in self.hsv_value_labels.items(): lab.setText(str(self.hsv_sliders[k].value()))
        self.lbl_hsv_summary.setText(
            f"H {hsv['h_min']}–{hsv['h_max']}   |   S {hsv['s_min']}–{hsv['s_max']}   |   "
            f"V {hsv['v_min']}–{hsv['v_max']}   |   Área {self.sp_min_area.value()}–{self.sp_max_area.value()} px²"
            + ('' if valid else '   ⚠ LIMITE MÍNIMO MAIOR QUE MÁXIMO')
        )
        if not valid: return
        amin=self.sp_min_area.value(); amax=self.sp_max_area.value()
        if amin > amax: return
        self.camera_worker.update_hsv(hsv)
        self.camera_worker.update_area_limits(amin,amax)
        self.cfg['hsv']=hsv; self.cfg['min_area']=amin; self.cfg['max_area']=amax

    def refresh_hsv_profiles(self, select_name=None):
        if not hasattr(self,'cb_hsv_profiles'): return
        self.cb_hsv_profiles.blockSignals(True); self.cb_hsv_profiles.clear()
        names=sorted(self.cfg.get('hsv_profiles',{}).keys())
        self.cb_hsv_profiles.addItems(names)
        if select_name in names: self.cb_hsv_profiles.setCurrentText(select_name)
        self.cb_hsv_profiles.blockSignals(False)

    def save_hsv_profile(self):
        name=self.ed_hsv_profile.text().strip().upper()
        if not name:
            QMessageBox.warning(self,'Perfil HSV','Informe um nome para o perfil, por exemplo LARANJA.')
            return
        self.apply_hsv_controls()
        hsv=self.current_hsv_from_controls()
        if hsv['h_min']>hsv['h_max'] or hsv['s_min']>hsv['s_max'] or hsv['v_min']>hsv['v_max']:
            QMessageBox.warning(self,'Perfil HSV','Corrija os limites HSV: mínimo não pode ser maior que máximo.')
            return
        if self.sp_min_area.value()>self.sp_max_area.value():
            QMessageBox.warning(self,'Perfil HSV','Área mínima não pode ser maior que a área máxima.')
            return
        self.cfg.setdefault('hsv_profiles',{})[name]={
            'hsv':hsv,
            'min_area':self.sp_min_area.value(),
            'max_area':self.sp_max_area.value()
        }
        ConfigManager.save(self.cfg); self.refresh_hsv_profiles(name)
        self.txt_console.append(f'[HSV] Perfil {name} salvo.')
        QMessageBox.information(self,'Perfil HSV',f'Perfil {name} salvo com sucesso.')

    def load_hsv_profile(self):
        name=self.cb_hsv_profiles.currentText().strip()
        profile=self.cfg.get('hsv_profiles',{}).get(name)
        if not profile:
            QMessageBox.warning(self,'Perfil HSV','Selecione um perfil salvo.')
            return
        hsv=profile.get('hsv',{})
        for k,sl in self.hsv_sliders.items():
            if k in hsv: sl.setValue(int(hsv[k]))
        self.sp_min_area.setValue(int(profile.get('min_area',500)))
        self.sp_max_area.setValue(int(profile.get('max_area',100000)))
        self.apply_hsv_controls()
        self.txt_console.append(f'[HSV] Perfil {name} carregado.')

    def delete_hsv_profile(self):
        name=self.cb_hsv_profiles.currentText().strip()
        if not name: return
        ans=QMessageBox.question(self,'Excluir perfil',f'Excluir o perfil HSV {name}?',QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
        if ans!=QMessageBox.Yes: return
        self.cfg.get('hsv_profiles',{}).pop(name,None)
        ConfigManager.save(self.cfg); self.refresh_hsv_profiles()
        self.txt_console.append(f'[HSV] Perfil {name} excluído.')

    def build_placeholder(self,title,text):
        w,l=self.page_shell(title,'Estrutura reservada na arquitetura DELTA VISION CONTROL.')
        card=QLabel('EM DESENVOLVIMENTO\n\n'+text); card.setAlignment(Qt.AlignCenter); card.setWordWrap(True); card.setStyleSheet('background:#1c2430;border:1px solid #303b4a;border-radius:12px;padding:35px;color:#9aa7b7;font-size:12pt;'); l.addWidget(card,1)

    def build_control(self):
        w,l=self.page_shell('TEACH / JOG + VISÃO',
            'Movimento manual cartesiano supervisionado pelo M3.9. Z é informado como magnitude positiva e tratado internamente como negativo.')

        main=QHBoxLayout()
        main.setSpacing(12)

        vision=QVBoxLayout()
        vision.setSpacing(10)

        self.lbl_cam_teach=QLabel('Aguardando câmera...')
        self.lbl_cam_teach.setAlignment(Qt.AlignCenter)
        self.lbl_cam_teach.setMinimumSize(540,390)
        self.lbl_cam_teach.setStyleSheet(
            'background:#000;'
            'border:1px solid #416fff;'
            'border-radius:10px;'
        )
        vision.addWidget(self.lbl_cam_teach,1)

        self.lbl_teach_objects=QLabel('OBJETOS: aguardando visão...')
        self.lbl_teach_objects.setWordWrap(True)
        self.lbl_teach_objects.setMinimumHeight(64)
        self.lbl_teach_objects.setStyleSheet(
            'background:#1c2430;'
            'border-radius:8px;'
            'padding:10px;'
            'font-size:10pt;'
        )
        vision.addWidget(self.lbl_teach_objects)

        main.addLayout(vision,5)

        panel=QVBoxLayout()
        panel.setSpacing(7)

        # --------------------------------------------------------------
        # REFERÊNCIA
        # --------------------------------------------------------------
        status=QGroupBox('Referência cartesiana do TEACH')
        sg=QGridLayout(status)
        sg.setHorizontalSpacing(12)
        sg.setVerticalSpacing(6)

        self.lbl_teach_xyz=QLabel('X 0.0 mm   Y 0.0 mm   Z ferramenta -600.0 mm')
        self.lbl_teach_xyz.setAlignment(Qt.AlignCenter)
        self.lbl_teach_xyz.setMinimumHeight(34)
        self.lbl_teach_xyz.setStyleSheet(
            'font-size:13pt;'
            'font-weight:700;'
            'background:#111821;'
            'border-radius:8px;'
            'padding:7px;'
        )

        self.lbl_teach_base_z=QLabel('Z base comandado: -600.0 mm')
        self.lbl_teach_base_z.setAlignment(Qt.AlignCenter)
        self.lbl_teach_base_z.setStyleSheet('color:#9fb7ff;font-weight:600;')

        self.lbl_teach_tool=QLabel('FERRAMENTA: OFF')
        self.lbl_teach_tool.setAlignment(Qt.AlignCenter)
        self.lbl_teach_tool.setMinimumHeight(26)
        self.lbl_teach_tool.setStyleSheet('font-size:10pt;font-weight:700;')

        self.lbl_teach_note=QLabel('Referência lógica da ponta da ferramenta; atualizada somente após OK do MOVEJ.')
        self.lbl_teach_note.setWordWrap(True)
        self.lbl_teach_note.setAlignment(Qt.AlignCenter)

        sg.addWidget(self.lbl_teach_xyz,0,0,1,3)
        sg.addWidget(self.lbl_teach_base_z,1,0,1,3)
        sg.addWidget(self.lbl_teach_tool,2,0,1,3)
        sg.addWidget(self.lbl_teach_note,3,0,1,3)
        panel.addWidget(status)

        # --------------------------------------------------------------
        # GEOMETRIA / SEGURANÇA Z
        # --------------------------------------------------------------
        zcfg=QGroupBox('Geometria da ferramenta / Limite Z')
        zg=QGridLayout(zcfg)
        zg.setHorizontalSpacing(8)
        zg.setVerticalSpacing(6)

        self.sp_tool_offset=QDoubleSpinBox()
        self.sp_tool_offset.setRange(0.0,300.0)
        self.sp_tool_offset.setDecimals(2)
        self.sp_tool_offset.setSuffix(' mm')
        self.sp_tool_offset.setValue(self.tool_offset_z)
        self.sp_tool_offset.setMinimumHeight(29)

        self.sp_z_min_limit=QDoubleSpinBox()
        self.sp_z_min_limit.setRange(0.0,900.0)
        self.sp_z_min_limit.setDecimals(2)
        self.sp_z_min_limit.setSuffix(' mm')
        self.sp_z_min_limit.setValue(abs(self.z_min_limit))
        self.sp_z_min_limit.setMinimumHeight(29)

        self.btn_save_zcfg=QPushButton('SALVAR OFFSET / LIMITE Z')
        self.btn_save_zcfg.setMinimumHeight(32)

        zhint=QLabel(
            'Os valores acima são digitados positivos. '
            'O sistema aplica Z negativo automaticamente. '
            'O limite é verificado na ponta da ferramenta.'
        )
        zhint.setWordWrap(True)
        zhint.setStyleSheet('color:#9aa7b7;')

        zg.addWidget(QLabel('Offset da ferramenta:'),0,0)
        zg.addWidget(self.sp_tool_offset,0,1)
        zg.addWidget(QLabel('Z limite mínimo:'),1,0)
        zg.addWidget(self.sp_z_min_limit,1,1)
        zg.addWidget(self.btn_save_zcfg,2,0,1,2)
        zg.addWidget(zhint,3,0,1,2)
        panel.addWidget(zcfg)

        # --------------------------------------------------------------
        # PASSO / JOG
        # --------------------------------------------------------------
        stepgrp=QGroupBox('Passo do JOG')
        stl=QHBoxLayout(stepgrp)
        stl.setSpacing(10)

        self.cb_jog_step=QComboBox()
        self.cb_jog_step.setMinimumHeight(30)
        self.cb_jog_step.setMinimumWidth(130)
        for v in (1,5,10,20):
            self.cb_jog_step.addItem(f'{v} mm',float(v))
        self.cb_jog_step.setCurrentIndex(2)

        stl.addWidget(QLabel('Incremento:'))
        stl.addWidget(self.cb_jog_step,1)
        panel.addWidget(stepgrp)

        jog=QGroupBox('JOG cartesiano')
        jg=QGridLayout(jog)
        jg.setHorizontalSpacing(8)
        jg.setVerticalSpacing(8)
        jg.setContentsMargins(10,12,10,12)

        self.btn_jog_yp=QPushButton('+Y')
        self.btn_jog_ym=QPushButton('-Y')
        self.btn_jog_xm=QPushButton('-X')
        self.btn_jog_xp=QPushButton('+X')
        self.btn_jog_zp=QPushButton('+Z')
        self.btn_jog_zm=QPushButton('-Z')

        jog_style=(
            'QPushButton {'
            'min-width:70px;'
            'min-height:40px;'
            'font-size:12pt;'
            'font-weight:700;'
            'border-radius:9px;'
            'padding:5px;'
            '}'
        )
        for b in (self.btn_jog_yp,self.btn_jog_ym,self.btn_jog_xm,
                  self.btn_jog_xp,self.btn_jog_zp,self.btn_jog_zm):
            b.setStyleSheet(jog_style)

        jg.addWidget(self.btn_jog_yp,0,1)
        jg.addWidget(self.btn_jog_xm,1,0)

        center=QLabel('XY')
        center.setAlignment(Qt.AlignCenter)
        center.setMinimumSize(56,40)
        center.setStyleSheet(
            'background:#111821;'
            'border-radius:8px;'
            'font-size:11pt;'
            'font-weight:700;'
            'color:#8fa8ff;'
        )
        jg.addWidget(center,1,1)
        jg.addWidget(self.btn_jog_xp,1,2)
        jg.addWidget(self.btn_jog_ym,2,1)

        zlabel=QLabel('EIXO Z')
        zlabel.setAlignment(Qt.AlignCenter)
        zlabel.setStyleSheet('font-weight:700;color:#8fa8ff;')
        jg.addWidget(zlabel,0,3)
        jg.addWidget(self.btn_jog_zp,1,3)
        jg.addWidget(self.btn_jog_zm,2,3)
        panel.addWidget(jog)

        # --------------------------------------------------------------
        # FERRAMENTA
        # --------------------------------------------------------------
        tool=QGroupBox('Ferramenta')
        tg=QHBoxLayout(tool)
        tg.setSpacing(10)

        self.btn_tool_on=QPushButton('LIGAR FERRAMENTA')
        self.btn_tool_off=QPushButton('DESLIGAR FERRAMENTA')
        for b in (self.btn_tool_on,self.btn_tool_off):
            b.setMinimumHeight(31)
            b.setStyleSheet(
                'font-size:10pt;'
                'font-weight:700;'
                'border-radius:8px;'
                'padding:7px 10px;'
            )
        tg.addWidget(self.btn_tool_on)
        tg.addWidget(self.btn_tool_off)
        panel.addWidget(tool)

        # --------------------------------------------------------------
        # MOVEJ ABSOLUTO
        # --------------------------------------------------------------
        absgrp=QGroupBox('MOVEJ absoluto / sincronizar referência')
        ag=QGridLayout(absgrp)
        ag.setHorizontalSpacing(8)
        ag.setVerticalSpacing(6)

        self.sp_x=QDoubleSpinBox()
        self.sp_y=QDoubleSpinBox()
        self.sp_z=QDoubleSpinBox()

        for sp in (self.sp_x,self.sp_y):
            sp.setRange(-500,500)
            sp.setDecimals(2)
            sp.setMinimumHeight(29)

        # Usuário digita magnitude positiva; internamente vira Z negativo.
        self.sp_z.setRange(0,900)
        self.sp_z.setValue(600)
        self.sp_z.setDecimals(2)
        self.sp_z.setSuffix(' mm')
        self.sp_z.setMinimumHeight(29)

        self.cb_tool=QComboBox()
        self.cb_tool.addItems(['OFF','ON'])
        self.cb_tool.setMinimumHeight(29)

        self.btn_movej=QPushButton('▶ EXECUTAR MOVEJ')
        self.btn_movej.setObjectName('start')
        self.btn_movej.setMinimumHeight(34)

        self.btn_sync_teach=QPushButton('USAR XYZ ACIMA COMO REFERÊNCIA')
        self.btn_sync_teach.setMinimumHeight(32)

        ag.addWidget(QLabel('X'),0,0)
        ag.addWidget(QLabel('Y'),0,1)
        ag.addWidget(QLabel('Z (magnitude)'),0,2)
        ag.addWidget(self.sp_x,1,0)
        ag.addWidget(self.sp_y,1,1)
        ag.addWidget(self.sp_z,1,2)
        ag.addWidget(QLabel('Ferramenta'),2,0)
        ag.addWidget(self.cb_tool,2,1,1,2)
        ag.addWidget(self.btn_movej,3,0,1,3)
        ag.addWidget(self.btn_sync_teach,4,0,1,3)
        panel.addWidget(absgrp)

        # --------------------------------------------------------------
        # HOLDING / ESPERA
        # --------------------------------------------------------------
        holdgrp=QGroupBox('Posição HOLDING / ESPERA')
        hg=QGridLayout(holdgrp)
        hg.setHorizontalSpacing(8)
        hg.setVerticalSpacing(6)

        self.sp_hold_x=QDoubleSpinBox()
        self.sp_hold_y=QDoubleSpinBox()
        self.sp_hold_z=QDoubleSpinBox()
        for sp in (self.sp_hold_x,self.sp_hold_y):
            sp.setRange(-500,500)
            sp.setDecimals(2)
            sp.setMinimumHeight(29)
        self.sp_hold_z.setRange(0,900)
        self.sp_hold_z.setDecimals(2)
        self.sp_hold_z.setSuffix(' mm')
        self.sp_hold_z.setMinimumHeight(29)

        self.cb_hold_tool=QComboBox()
        self.cb_hold_tool.addItems(['OFF','ON'])
        self.cb_hold_tool.setMinimumHeight(29)

        hcfg=self.holding
        self.sp_hold_x.setValue(float(hcfg.get('x',0.0)))
        self.sp_hold_y.setValue(float(hcfg.get('y',0.0)))
        self.sp_hold_z.setValue(abs(float(hcfg.get('z',-550.0))))
        self.cb_hold_tool.setCurrentText(str(hcfg.get('tool','OFF')))

        self.btn_save_holding=QPushButton('SALVAR HOLDING')
        self.btn_go_holding=QPushButton('IR PARA HOLDING')
        self.btn_go_holding.setObjectName('start')
        for b in (self.btn_save_holding,self.btn_go_holding):
            b.setMinimumHeight(32)

        holdhint=QLabel(
            'Use esta posição para retirar o robô do campo de visão da câmera. '
            'Ela ficará disponível para o futuro sequenciador automático.'
        )
        holdhint.setWordWrap(True)
        holdhint.setStyleSheet('color:#9aa7b7;')

        hg.addWidget(QLabel('X'),0,0)
        hg.addWidget(QLabel('Y'),0,1)
        hg.addWidget(QLabel('Z (magnitude)'),0,2)
        hg.addWidget(self.sp_hold_x,1,0)
        hg.addWidget(self.sp_hold_y,1,1)
        hg.addWidget(self.sp_hold_z,1,2)
        hg.addWidget(QLabel('Ferramenta'),2,0)
        hg.addWidget(self.cb_hold_tool,2,1,1,2)
        hg.addWidget(self.btn_save_holding,3,0,1,3)
        hg.addWidget(self.btn_go_holding,4,0,1,3)
        hg.addWidget(holdhint,5,0,1,3)
        panel.addWidget(holdgrp)

        # --------------------------------------------------------------
        # PICK & PLACE SEMIASSISTIDO
        # --------------------------------------------------------------
        ppgrp=QGroupBox('PICK & PLACE — 1 PEÇA')
        pg=QGridLayout(ppgrp)
        pg.setHorizontalSpacing(8)
        pg.setVerticalSpacing(6)

        pp=self.pick_place
        self.sp_pp_approach=QDoubleSpinBox(); self.sp_pp_approach.setRange(0,900); self.sp_pp_approach.setSuffix(' mm'); self.sp_pp_approach.setValue(abs(float(pp.get('approach_z',-550))))
        self.sp_pp_pick=QDoubleSpinBox(); self.sp_pp_pick.setRange(0,900); self.sp_pp_pick.setSuffix(' mm'); self.sp_pp_pick.setValue(abs(float(pp.get('pick_z',-650))))
        self.sp_pp_safe=QDoubleSpinBox(); self.sp_pp_safe.setRange(0,900); self.sp_pp_safe.setSuffix(' mm'); self.sp_pp_safe.setValue(abs(float(pp.get('safe_z',-500))))
        self.sp_pp_drop_x=QDoubleSpinBox(); self.sp_pp_drop_x.setRange(-500,500); self.sp_pp_drop_x.setValue(float(pp.get('drop_x',150)))
        self.sp_pp_drop_y=QDoubleSpinBox(); self.sp_pp_drop_y.setRange(-500,500); self.sp_pp_drop_y.setValue(float(pp.get('drop_y',0)))
        self.sp_pp_drop_z=QDoubleSpinBox(); self.sp_pp_drop_z.setRange(0,900); self.sp_pp_drop_z.setSuffix(' mm'); self.sp_pp_drop_z.setValue(abs(float(pp.get('drop_z',-650))))
        for sp in (self.sp_pp_approach,self.sp_pp_pick,self.sp_pp_safe,self.sp_pp_drop_x,self.sp_pp_drop_y,self.sp_pp_drop_z): sp.setMinimumHeight(29)

        self.btn_pp_save=QPushButton('SALVAR ALTURAS / DROP')
        self.btn_pp_capture=QPushButton('CAPTURAR ÚNICO OBJETO STABLE')
        self.btn_pp_cycle=QPushButton('▶ EXECUTAR 1 CICLO')
        self.btn_pp_cycle.setObjectName('start')
        self.btn_pp_abort=QPushButton('CANCELAR SEQUÊNCIA')
        for b in (self.btn_pp_save,self.btn_pp_capture,self.btn_pp_cycle,self.btn_pp_abort): b.setMinimumHeight(32)

        self.lbl_pp_target=QLabel('ALVO: não capturado')
        self.lbl_pp_target.setWordWrap(True)
        self.lbl_pp_target.setStyleSheet('background:#111821;border-radius:7px;padding:8px;font-weight:600;')
        self.lbl_pp_status=QLabel('SEQUÊNCIA: parada')
        self.lbl_pp_status.setWordWrap(True)
        self.lbl_pp_status.setStyleSheet('color:#a8bcff;padding:5px;')

        pg.addWidget(QLabel('Z aproximação'),0,0); pg.addWidget(self.sp_pp_approach,0,1)
        pg.addWidget(QLabel('Z PICK'),1,0); pg.addWidget(self.sp_pp_pick,1,1)
        pg.addWidget(QLabel('Z seguro'),2,0); pg.addWidget(self.sp_pp_safe,2,1)
        pg.addWidget(QLabel('DROP X'),3,0); pg.addWidget(self.sp_pp_drop_x,3,1)
        pg.addWidget(QLabel('DROP Y'),4,0); pg.addWidget(self.sp_pp_drop_y,4,1)
        pg.addWidget(QLabel('DROP Z'),5,0); pg.addWidget(self.sp_pp_drop_z,5,1)
        pg.addWidget(self.btn_pp_save,6,0,1,2)
        pg.addWidget(self.btn_pp_capture,7,0,1,2)
        pg.addWidget(self.lbl_pp_target,8,0,1,2)
        pg.addWidget(self.btn_pp_cycle,9,0,1,2)
        pg.addWidget(self.btn_pp_abort,10,0,1,2)
        pg.addWidget(self.lbl_pp_status,11,0,1,2)
        ppnote=QLabel('MVP: o ciclo só inicia com exatamente 1 objeto STABLE capturado. O alvo X/Y é congelado antes do movimento.')
        ppnote.setWordWrap(True); ppnote.setStyleSheet('color:#9aa7b7;')
        pg.addWidget(ppnote,12,0,1,2)
        panel.addWidget(ppgrp)

        # --------------------------------------------------------------
        # CONTROLADOR
        # --------------------------------------------------------------
        cmds=QGroupBox('Controlador M3.9')
        cg=QGridLayout(cmds)
        cg.setHorizontalSpacing(8)
        cg.setVerticalSpacing(6)

        self.cb_ports=QComboBox()
        self.cb_ports.setMinimumHeight(29)

        self.btn_refresh_ports=QPushButton('SCAN')
        self.btn_connect=QPushButton('CONECTAR')
        self.btn_disconnect=QPushButton('DESCONECTAR')
        self.btn_status=QPushButton('STATUS')
        self.btn_home=QPushButton('HOME')
        self.btn_reset=QPushButton('RESET')
        self.btn_reset.setObjectName('reset')

        for b in (self.btn_refresh_ports,self.btn_connect,self.btn_disconnect,
                  self.btn_status,self.btn_home,self.btn_reset):
            b.setMinimumHeight(31)

        cg.addWidget(self.cb_ports,0,0,1,3)
        cg.addWidget(self.btn_refresh_ports,1,0)
        cg.addWidget(self.btn_connect,1,1)
        cg.addWidget(self.btn_disconnect,1,2)
        cg.addWidget(self.btn_status,2,0)
        cg.addWidget(self.btn_home,2,1)
        cg.addWidget(self.btn_reset,2,2)
        panel.addWidget(cmds)

        self.lbl_interlocks=QLabel(
            '✕ ESP32 desconectado\n'
            '✕ Estado READY não confirmado\n'
            '✕ JOG bloqueado'
        )
        self.lbl_interlocks.setWordWrap(True)
        self.lbl_interlocks.setMinimumHeight(62)
        self.lbl_interlocks.setStyleSheet(
            'background:#1c2430;'
            'border-radius:8px;'
            'padding:10px;'
            'font-size:10pt;'
        )
        panel.addWidget(self.lbl_interlocks)
        panel.addStretch()

        # Painel vertical deslizante. Nada é sobreposto quando a janela é menor.
        panel_host=QWidget()
        panel_host.setLayout(panel)
        panel_scroll=QScrollArea()
        panel_scroll.setWidgetResizable(True)
        panel_scroll.setFrameShape(QFrame.NoFrame)
        panel_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        panel_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        panel_scroll.setWidget(panel_host)
        panel_scroll.setMinimumWidth(440)
        main.addWidget(panel_scroll,4)

        l.addLayout(main,1)

        self.jog_buttons=[
            self.btn_jog_yp,self.btn_jog_ym,
            self.btn_jog_xm,self.btn_jog_xp,
            self.btn_jog_zp,self.btn_jog_zm,
            self.btn_tool_on,self.btn_tool_off,
            self.btn_go_holding
        ]

    def build_manual(self):
        w,l=self.page_shell('MANUAL & TUTORIAL','Manual integrado de operação, protocolo, estados e recuperação de falhas.')
        self.manual_search=QLineEdit(); self.manual_search.setPlaceholderText('Pesquisar: STOP, MOVEJ, HOME, KINEMATICS...'); l.addWidget(self.manual_search)
        self.manual_browser=QTextEdit(); self.manual_browser.setReadOnly(True); self.manual_browser.setObjectName('manual_browser'); self.manual_text="""DELTA VISION CONTROL — GUIA RÁPIDO M3.9

PRIMEIROS PASSOS
1. Conecte o ESP32 #2 via USB.
2. Clique SCAN e selecione a porta.
3. Clique CONECTAR.
4. Consulte STATUS.
5. Em NOT_HOMED, execute HOME.
6. Aguarde READY antes de MOVEJ.

MÁQUINA DE ESTADOS
NOT_HOMED → HOME → HOMING → READY → MOVEJ → MOVING
MOVING → PAUSE → PAUSED → RESUME → MOVING
MOVING/PAUSED → STOP → STOPPED → confirmação do operador → RESET → NOT_HOMED
READY/MOVING → EMERGENCY|ACTIVE → EMERGENCY
EMERGENCY → liberação física → EMERGENCY|RELEASED → confirmação do operador → RESET → NOT_HOMED
NOT_HOMED → HOME → HOMING → READY

COMANDOS
STATUS  | consulta o estado atual
HOME    | executa referenciamento
MOVEJ   | CMD|0001|MOVEJ|X|Y|Z|ON/OFF
PAUSE   | pausa movimento ativo
RESUME  | retoma movimento pausado
STOP    | parada operacional; exige rearme supervisório
RESET   | rearme lógico; após STOP/EMERGENCY exige confirmação na HMI

EMERGÊNCIA M3.9
EMERGENCY|ACTIVE — emergência física acionada; movimentos ficam bloqueados.
EMERGENCY|RELEASED — contato físico liberado; o robô continua aguardando rearme.
EMERGENCY_STOP — movimento ativo foi interrompido pela emergência.
EMERGENCY_ACTIVE — RESET recusado enquanto a emergência física estiver ativa.

ERROS IMPORTANTES
ROBOT_NOT_READY — robô não está autorizado a mover.
JOINT_LIMIT — movimento viola limite angular.
KINEMATICS — coordenada não possui solução cinemática válida.
LIMIT_SWITCH — fim de curso acionado durante operação.
HOMING_TIMEOUT — homing não terminou no tempo esperado.
STOPPED_BY_OPERATOR — movimento foi interrompido por STOP.
UNKNOWN_COMMAND — comando desconhecido.
INVALID_FRAME — quadro serial inválido.


VISÃO / PERFIS HSV
Na aba CORES, ajuste H/S/V mínimo e máximo observando a máscara em tempo real.
Branco na máscara = região aceita pelo detector; preto = região descartada.
Use Área mínima/máxima para rejeitar ruídos ou objetos grandes indesejados.
Perfis HSV podem ser salvos com nomes como LARANJA e recarregados nos testes.
A calibração por homografia e o HSV são funções diferentes: HSV segmenta o objeto; homografia converte pixel para X/Y.

PICK & PLACE SEMIASSISTIDO — 1 PEÇA
Capture exatamente um objeto STABLE. A HMI congela o X/Y do alvo e executa um único ciclo supervisionado: aproximação, PICK, ferramenta ON, subida segura, DROP, ferramenta OFF, subida e HOLDING.
O ciclo é cancelado por erros de protocolo ou estados de bloqueio. STOP e EMERGENCY do M3.9 permanecem soberanos.

TEACH / OFFSET / HOLDING
Na aba TEACH, valores Z digitados pelo usuário são magnitudes positivas e são convertidos internamente para Z negativo.
O campo Offset da ferramenta representa quanto a ponta está abaixo da base móvel.
Exemplo: Z da ponta = -650 mm e offset = 80 mm → comando enviado à base = -570 mm.
O Z limite mínimo é verificado sobre a ponta da ferramenta antes do envio do MOVEJ.
A posição HOLDING é persistente e pode ser usada para retirar o robô do campo de visão da câmera.

SEGURANÇA
A proteção de Z da HMI é supervisória e complementar; o firmware M3.9 continua soberano sobre cinemática e limites.
A confirmação da HMI é supervisória e não constitui função de segurança.
A emergência física continua atuando independentemente sobre o ENA dos drivers e é monitorada pelo GPIO21.
A interface não substitui os elementos físicos de segurança. O firmware M3.9 continua sendo a autoridade sobre estados, cinemática, limites e execução física."""
        self.manual_browser.setPlainText(self.manual_text); l.addWidget(self.manual_browser,1)

    def build_diagnostics(self):
        w,l=self.page_shell('DIAGNÓSTICO','Terminal bruto do protocolo M3.9 para validação e manutenção.')
        self.txt_console=QTextEdit(); self.txt_console.setReadOnly(True); l.addWidget(self.txt_console,1); clear=QPushButton('LIMPAR TERMINAL'); clear.clicked.connect(self.txt_console.clear); l.addWidget(clear)

    def connect_signals(self):
        self.btn_go_control.clicked.connect(lambda:self.show_page(7)); self.btn_refresh_ports.clicked.connect(self.refresh_ports); self.btn_connect.clicked.connect(self.connect_port); self.btn_disconnect.clicked.connect(self.serial_worker.disconnect_serial)
        self.btn_status.clicked.connect(lambda:self.serial_worker.send_cmd('STATUS')); self.btn_home.clicked.connect(lambda:self.send_m38('HOME')); self.btn_reset.clicked.connect(self.request_supervisory_reset); self.btn_movej.clicked.connect(self.send_movej); self.btn_pause.clicked.connect(lambda:self.send_m38('PAUSE')); self.btn_resume.clicked.connect(lambda:self.send_m38('RESUME')); self.btn_stop.clicked.connect(lambda:self.send_m38('STOP'))
        self.cb_cameras.currentIndexChanged.connect(self.change_camera); self.lbl_cam_calib.sig_clicked_coords.connect(self.on_calib_click); self.btn_reset_calib.clicked.connect(self.reset_calib_points); self.btn_save_homography.clicked.connect(self.calculate_homography); self.manual_search.textChanged.connect(self.search_manual)
        for sl in self.hsv_sliders.values(): sl.valueChanged.connect(self.apply_hsv_controls)
        self.sp_min_area.valueChanged.connect(self.apply_hsv_controls); self.sp_max_area.valueChanged.connect(self.apply_hsv_controls)
        self.btn_hsv_save.clicked.connect(self.save_hsv_profile); self.btn_hsv_load.clicked.connect(self.load_hsv_profile); self.btn_hsv_delete.clicked.connect(self.delete_hsv_profile)
        for w in (self.sp_stable_frames,self.sp_match_px,self.sp_stable_px,self.sp_missed,self.sp_recovery_px,self.sp_area_tol):
            w.valueChanged.connect(self.apply_tracking_controls)
        self.btn_reset_tracks.clicked.connect(self.camera_worker.reset_tracks)
        self.btn_jog_xp.clicked.connect(lambda:self.jog_axis('X',+1))
        self.btn_jog_xm.clicked.connect(lambda:self.jog_axis('X',-1))
        self.btn_jog_yp.clicked.connect(lambda:self.jog_axis('Y',+1))
        self.btn_jog_ym.clicked.connect(lambda:self.jog_axis('Y',-1))
        self.btn_jog_zp.clicked.connect(lambda:self.jog_axis('Z',+1))
        self.btn_jog_zm.clicked.connect(lambda:self.jog_axis('Z',-1))
        self.btn_tool_on.clicked.connect(lambda:self.set_teach_tool('ON'))
        self.btn_tool_off.clicked.connect(lambda:self.set_teach_tool('OFF'))
        self.btn_sync_teach.clicked.connect(self.sync_teach_reference)
        self.btn_save_zcfg.clicked.connect(self.save_z_settings)
        self.btn_save_holding.clicked.connect(self.save_holding)
        self.btn_go_holding.clicked.connect(self.go_holding)
        self.btn_pp_save.clicked.connect(self.save_pick_place)
        self.btn_pp_capture.clicked.connect(self.capture_pick_target)
        self.btn_pp_cycle.clicked.connect(self.start_pick_place_cycle)
        self.btn_pp_abort.clicked.connect(lambda:self.cancel_pick_place_cycle('Cancelada pelo operador.'))
        self.serial_worker.sig_state_changed.connect(self.on_robot_state_changed); self.serial_worker.sig_log.connect(self.txt_console.append); self.serial_worker.sig_command_done.connect(self.on_command_finished); self.serial_worker.sig_error_received.connect(self.on_protocol_error); self.serial_worker.sig_boot_ready.connect(self.on_controller_boot_ready); self.serial_worker.sig_reboot_detected.connect(self.on_controller_reboot_detected)
        self.camera_worker.sig_frame_processed.connect(self.update_main_camera); self.camera_worker.sig_calib_frame.connect(self.update_calib_camera); self.camera_worker.sig_target_detected.connect(self.on_target_detected); self.camera_worker.sig_objects.connect(self.update_objects_table)

    def show_page(self,idx):
        self.stack.setCurrentIndex(idx)
        for i,b in enumerate(self.nav_buttons): b.setChecked(i==idx)

    def populate_camera_list(self):
        self.cb_cameras.blockSignals(True); self.cb_cameras.clear()
        for i in range(3): self.cb_cameras.addItem(f'Câmera {i}',i)
        pos=self.cb_cameras.findData(self.cfg.get('camera_index',0)); self.cb_cameras.setCurrentIndex(max(0,pos)); self.cb_cameras.blockSignals(False)

    def change_camera(self,index):
        cam=self.cb_cameras.itemData(index)
        if cam is not None: self.camera_worker.set_camera_index(cam); self.cfg['camera_index']=cam; self.txt_console.append(f'[VÍDEO] Câmera {cam} selecionada.')

    def refresh_ports(self):
        current=self.cb_ports.currentText(); self.cb_ports.clear()
        for p in serial_list_ports.comports(): self.cb_ports.addItem(p.device)
        saved=self.cfg.get('port','') or current; idx=self.cb_ports.findText(saved)
        if idx>=0:self.cb_ports.setCurrentIndex(idx)

    def connect_port(self):
        port=self.cb_ports.currentText()
        if not port:
            QMessageBox.warning(self,'Serial','Nenhuma porta serial encontrada.')
            return
        if self.serial_worker.connect_serial(port):
            self.cfg['port']=port
            self.initial_status_sent = False
            # Fallback robusto: alguns drivers/CP210x/CH340 podem não entregar
            # o banner inicial do ESP32 à aplicação. Depois de 1,8 s fazemos
            # uma única consulta STATUS, apenas se o boot não tiver sincronizado antes.
            QTimer.singleShot(1800, self.ensure_initial_status)

    def ensure_initial_status(self):
        if self.initial_status_sent:
            return
        if not self.serial_worker.ser or not self.serial_worker.ser.is_open:
            return
        self.initial_status_sent = True
        self.txt_console.append('[SYNC] Consultando estado inicial do controlador...')
        self.serial_worker.send_cmd('STATUS')

    def on_controller_reboot_detected(self):
        # O ESP32 pode reiniciar mantendo a porta COM aberta (RESET físico / ESP.restart).
        # Nesse caso precisamos considerar o estado anterior inválido e refazer o STATUS.
        self.initial_status_sent = False
        self.serial_worker.current_state = 'RESTARTING'
        self.serial_worker.sig_state_changed.emit('RESTARTING')
        self.txt_console.append('[SYNC] Reinicialização do controlador detectada. Aguardando M3.9...')

    def on_controller_boot_ready(self):
        if self.initial_status_sent:
            return
        self.txt_console.append('[SYNC] Firmware M3.9 detectado. Consultando STATUS...')
        QTimer.singleShot(250, self.ensure_initial_status)

    def send_m38(self,cmd,params=''):
        st=self.serial_worker.current_state
        allowed={'STATUS':None,'STOP':None,'HOME':{'NOT_HOMED','READY'},'MOVEJ':{'READY'},'PAUSE':{'MOVING'},'RESUME':{'PAUSED'},'RESET':None}
        req=allowed.get(cmd)
        if req is not None and st not in req:
            QMessageBox.warning(self,'Intertravamento M3.9',f'{cmd} bloqueado pela interface no estado {st}.'); return -1
        cid=self.serial_worker.send_cmd(cmd,params); self.last_cmd_id=cid; return cid

    def request_supervisory_reset(self):
        st = self.serial_worker.current_state

        if st == 'EMERGENCY':
            QMessageBox.warning(
                self, 'Emergência ainda ativa',
                'A emergência física continua ATIVA.\n\n'
                'Libere a botoeira e aguarde EMERGENCY|RELEASED antes de solicitar o rearme.'
            )
            return -1

        if st not in ('STOPPED', 'EMERGENCY_RELEASED'):
            return self.send_m38('RESET')

        origem = 'STOPPED' if st == 'STOPPED' else 'EMERGÊNCIA LIBERADA'
        box = QMessageBox(self)
        box.setWindowTitle('Confirmação de rearme')
        box.setIcon(QMessageBox.Warning)
        box.setText(f'O robô encontra-se em {origem}.')
        box.setInformativeText(
            'Antes de continuar:\n'
            '• verifique fisicamente o robô e a área de trabalho;\n'
            '• confirme que não há pessoas ou obstáculos na área;\n'
            '• confirme que o robô pode ser reinicializado.\n\n'
            'Após o RESET, HOME será obrigatório.'
        )
        check = QCheckBox(
            'Verifiquei fisicamente o robô e a área de trabalho e desejo realizar o rearme.'
        )
        box.setCheckBox(check)
        box.setStandardButtons(QMessageBox.Cancel | QMessageBox.Yes)
        box.setDefaultButton(QMessageBox.Cancel)
        btn_rearm = box.button(QMessageBox.Yes)
        btn_rearm.setText('REARMAR')
        btn_rearm.setEnabled(False)
        box.button(QMessageBox.Cancel).setText('CANCELAR')
        check.toggled.connect(btn_rearm.setEnabled)

        result = box.exec_()
        if result == QMessageBox.Yes and check.isChecked():
            self.txt_console.append(f'[REARME] Confirmação supervisória aceita em {st}. Enviando RESET.')
            return self.send_m38('RESET')

        self.txt_console.append('[REARME] Operação cancelada; nenhum RESET foi enviado.')
        return -1

    def z_tool_to_base(self, tool_z):
        """Converte Z da ponta da ferramenta para Z da base móvel."""
        return float(tool_z) + float(self.tool_offset_z)

    def validate_tool_target(self, x, y, tool_z, show_message=True):
        """Validação supervisória da HMI. Firmware M3.9 continua soberano."""
        x=float(x); y=float(y); tool_z=float(tool_z)
        base_z=self.z_tool_to_base(tool_z)

        reason=None
        if not (-500.0 <= x <= 500.0 and -500.0 <= y <= 500.0):
            reason='X/Y fora dos limites de edição da HMI.'
        elif tool_z > 0.0:
            reason='Z da ferramenta deve ser zero ou negativo.'
        elif tool_z < self.z_min_limit:
            reason=(
                f'Z solicitado ({tool_z:.2f} mm) ultrapassa o limite mínimo '
                f'configurado ({self.z_min_limit:.2f} mm).'
            )
        elif not (-800.0 <= base_z <= 0.0):
            reason=(
                f'Com offset de {self.tool_offset_z:.2f} mm, o Z comandado da base '
                f'seria {base_z:.2f} mm, fora da faixa -800..0 mm.'
            )

        if reason and show_message:
            QMessageBox.warning(self,'Proteção Z / TEACH',reason)
        return reason is None

    def send_teach_target(self, x, y, tool_z, tool, source='MOVEJ'):
        """Envia MOVEJ usando coordenada lógica da ponta e aplica offset antes do protocolo."""
        x=float(x); y=float(y); tool_z=float(tool_z)
        if not self.validate_tool_target(x,y,tool_z):
            return -1

        base_z=self.z_tool_to_base(tool_z)
        cid=self.send_m38('MOVEJ',f'{x:.2f}|{y:.2f}|{base_z:.2f}|{tool}')
        if cid != -1:
            # A referência lógica preserva o Z da ponta, não o Z da base.
            self.teach_pending=(cid,x,y,tool_z,tool)
            self.txt_console.append(
                f'[TEACH] {source}: ponta X={x:.2f} Y={y:.2f} Z={tool_z:.2f} | '
                f'offset={self.tool_offset_z:.2f} -> base Z={base_z:.2f} | TOOL={tool}'
            )
        return cid

    def send_movej(self):
        x=float(self.sp_x.value())
        y=float(self.sp_y.value())
        tool_z=-abs(float(self.sp_z.value()))
        tool=self.cb_tool.currentText()
        return self.send_teach_target(x,y,tool_z,tool,'MOVEJ absoluto')

    def update_teach_reference_labels(self):
        if hasattr(self,'lbl_teach_xyz'):
            self.lbl_teach_xyz.setText(
                f'X {self.teach_x:.1f} mm   Y {self.teach_y:.1f} mm   '
                f'Z ferramenta {self.teach_z:.1f} mm'
            )
            self.lbl_teach_base_z.setText(
                f'Z base comandado: {self.z_tool_to_base(self.teach_z):.1f} mm   '
                f'(offset {self.tool_offset_z:.1f} mm)'
            )
            self.lbl_teach_tool.setText(f'FERRAMENTA: {self.teach_tool}')

    def save_z_settings(self):
        offset=abs(float(self.sp_tool_offset.value()))
        zmin=-abs(float(self.sp_z_min_limit.value()))

        # Faixa coerente com a edição atual da HMI.
        if offset > 300.0:
            QMessageBox.warning(self,'Configuração Z','Offset da ferramenta inválido.')
            return
        if zmin < -900.0 or zmin > 0.0:
            QMessageBox.warning(self,'Configuração Z','Limite mínimo Z inválido.')
            return

        self.tool_offset_z=offset
        self.z_min_limit=zmin
        self.cfg['tool_offset_z']=offset
        self.cfg['z_min_limit']=zmin
        ConfigManager.save(self.cfg)
        self.update_teach_reference_labels()
        self.txt_console.append(
            f'[CONFIG] Offset ferramenta={offset:.2f} mm | Z mínimo da ponta={zmin:.2f} mm'
        )
        QMessageBox.information(
            self,'Configuração Z',
            f'Configuração salva.\n\n'
            f'Offset da ferramenta: {offset:.2f} mm\n'
            f'Z limite mínimo da ponta: {zmin:.2f} mm'
        )

    def save_holding(self):
        x=float(self.sp_hold_x.value())
        y=float(self.sp_hold_y.value())
        z=-abs(float(self.sp_hold_z.value()))
        tool=self.cb_hold_tool.currentText()

        if not self.validate_tool_target(x,y,z):
            return

        self.holding={'x':x,'y':y,'z':z,'tool':tool}
        self.cfg['holding']=dict(self.holding)
        ConfigManager.save(self.cfg)
        self.txt_console.append(
            f'[HOLDING] Salvo: X={x:.2f} Y={y:.2f} Z ponta={z:.2f} TOOL={tool}'
        )
        QMessageBox.information(self,'HOLDING','Posição HOLDING salva com sucesso.')

    def go_holding(self):
        if self.serial_worker.current_state!='READY':
            QMessageBox.warning(
                self,'HOLDING',
                f'HOLDING bloqueado no estado {self.serial_worker.current_state}.'
            )
            return -1

        h=self.holding
        return self.send_teach_target(
            float(h.get('x',0.0)),
            float(h.get('y',0.0)),
            float(h.get('z',-550.0)),
            str(h.get('tool','OFF')),
            'HOLDING'
        )

    def sync_teach_reference(self):
        if self.serial_worker.current_state!='READY':
            QMessageBox.warning(self,'TEACH','Sincronização da referência permitida somente em READY.')
            return

        x=float(self.sp_x.value())
        y=float(self.sp_y.value())
        z=-abs(float(self.sp_z.value()))
        if not self.validate_tool_target(x,y,z):
            return

        self.teach_x=x
        self.teach_y=y
        self.teach_z=z
        self.teach_tool=self.cb_tool.currentText()
        self.teach_pending=None
        self.update_teach_reference_labels()
        self.txt_console.append(
            f'[TEACH] Referência lógica definida: '
            f'X={self.teach_x:.2f} Y={self.teach_y:.2f} '
            f'Z ponta={self.teach_z:.2f} TOOL={self.teach_tool}'
        )

    def jog_axis(self,axis,direction):
        if self.serial_worker.current_state!='READY':
            QMessageBox.warning(
                self,'JOG',
                f'JOG bloqueado no estado {self.serial_worker.current_state}.'
            )
            return

        step=float(self.cb_jog_step.currentData() or 10.0)
        x,y,z=self.teach_x,self.teach_y,self.teach_z

        if axis=='X':
            x += direction*step
        elif axis=='Y':
            y += direction*step
        elif axis=='Z':
            # +Z sobe (menos negativo); -Z desce (mais negativo).
            z += direction*step

        cid=self.send_teach_target(
            x,y,z,self.teach_tool,
            f'JOG {axis} {"+" if direction>0 else "-"}{step:g}'
        )
        return cid

    def set_teach_tool(self,state):
        if self.serial_worker.current_state!='READY':
            QMessageBox.warning(
                self,'TEACH',
                f'Ferramenta bloqueada no estado {self.serial_worker.current_state}.'
            )
            return

        # M3.9 controla TOOL dentro de MOVEJ. Reenvia a posição lógica atual,
        # convertendo Z da ponta em Z da base por meio do offset.
        self.send_teach_target(
            self.teach_x,self.teach_y,self.teach_z,state,
            f'TOOL={state}'
        )

    def save_pick_place(self):
        pp={
            'approach_z':-abs(float(self.sp_pp_approach.value())),
            'pick_z':-abs(float(self.sp_pp_pick.value())),
            'safe_z':-abs(float(self.sp_pp_safe.value())),
            'drop_x':float(self.sp_pp_drop_x.value()),
            'drop_y':float(self.sp_pp_drop_y.value()),
            'drop_z':-abs(float(self.sp_pp_drop_z.value()))
        }
        # Valida todos os Z na ponta da ferramenta antes de persistir.
        for label,x,y,z in (
            ('Z aproximação',0,0,pp['approach_z']),('Z PICK',0,0,pp['pick_z']),
            ('Z seguro',0,0,pp['safe_z']),('DROP',pp['drop_x'],pp['drop_y'],pp['drop_z'])):
            if not self.validate_tool_target(x,y,z,False):
                QMessageBox.warning(self,'PICK & PLACE',f'{label} viola o limite Z/área configurada.')
                return
        if not (pp['safe_z'] >= pp['approach_z'] >= pp['pick_z']):
            QMessageBox.warning(self,'PICK & PLACE','Use Z seguro mais alto, depois Z aproximação e Z PICK mais baixo.\nEx.: 500 / 550 / 650 mm na interface.')
            return
        self.pick_place=pp; self.cfg['pick_place']=dict(pp); ConfigManager.save(self.cfg)
        self.txt_console.append(f"[PICKPLACE] Configuração salva: {pp}")
        QMessageBox.information(self,'PICK & PLACE','Alturas e posição DROP salvas.')

    def capture_pick_target(self):
        stable=[o for o in self.last_objects if o.get('state')=='STABLE' and o.get('x') is not None and o.get('y') is not None]
        if len(stable)!=1:
            QMessageBox.warning(self,'Captura do alvo',f'É necessário exatamente 1 objeto STABLE.\nObjetos STABLE válidos agora: {len(stable)}')
            return
        o=stable[0]
        self.pick_target={'id':o.get('id'),'x':float(o['x']),'y':float(o['y']),'shape':o.get('shape','')}
        self.lbl_pp_target.setText(f"ALVO CONGELADO — ID {self.pick_target['id']} | {self.pick_target['shape']} | X {self.pick_target['x']:.1f} | Y {self.pick_target['y']:.1f} mm")
        self.txt_console.append(f"[PICKPLACE] Alvo congelado: {self.pick_target}")

    def start_pick_place_cycle(self):
        if self.pick_cycle_active:
            QMessageBox.warning(self,'PICK & PLACE','Já existe uma sequência em execução.')
            return
        if self.serial_worker.current_state!='READY':
            QMessageBox.warning(self,'PICK & PLACE','O controlador deve estar em READY.')
            return
        if not self.pick_target:
            QMessageBox.warning(self,'PICK & PLACE','Capture primeiro o único objeto STABLE.')
            return
        # Política MVP: também exige que continue existindo somente um STABLE no instante do START.
        stable=[o for o in self.last_objects if o.get('state')=='STABLE' and o.get('x') is not None]
        if len(stable)!=1:
            QMessageBox.warning(self,'PICK & PLACE','Ciclo bloqueado: deve existir exatamente 1 objeto STABLE no START.')
            return
        self.save_pick_place()
        pp=self.pick_place; t=self.pick_target; h=self.holding
        steps=[
            ('APROXIMAR PICK',t['x'],t['y'],pp['approach_z'],'OFF'),
            ('DESCER PICK',t['x'],t['y'],pp['pick_z'],'OFF'),
            ('FERRAMENTA ON',t['x'],t['y'],pp['pick_z'],'ON'),
            ('SUBIR PICK',t['x'],t['y'],pp['safe_z'],'ON'),
            ('IR SOBRE DROP',pp['drop_x'],pp['drop_y'],pp['safe_z'],'ON'),
            ('DESCER DROP',pp['drop_x'],pp['drop_y'],pp['drop_z'],'ON'),
            ('FERRAMENTA OFF',pp['drop_x'],pp['drop_y'],pp['drop_z'],'OFF'),
            ('SUBIR DROP',pp['drop_x'],pp['drop_y'],pp['safe_z'],'OFF'),
            ('HOLDING',float(h.get('x',0)),float(h.get('y',0)),float(h.get('z',-550)),'OFF'),
        ]
        for name,x,y,z,tool in steps:
            if not self.validate_tool_target(x,y,z,False):
                QMessageBox.warning(self,'PICK & PLACE',f'Sequência bloqueada: etapa {name} viola a proteção Z.')
                return
        self.pick_cycle_steps=steps; self.pick_cycle_index=0; self.pick_cycle_waiting_id=None; self.pick_cycle_active=True
        self.lbl_pp_status.setText('SEQUÊNCIA: iniciando...')
        self.txt_console.append('[PICKPLACE] === INÍCIO DE 1 CICLO ===')
        self._advance_pick_place_cycle()

    def _advance_pick_place_cycle(self):
        if not self.pick_cycle_active or self.pick_cycle_waiting_id is not None: return
        if self.serial_worker.current_state!='READY': return
        if self.pick_cycle_index>=len(self.pick_cycle_steps):
            self.pick_cycle_active=False; self.pick_target=None
            self.lbl_pp_status.setText('SEQUÊNCIA: CICLO CONCLUÍDO ✓')
            self.lbl_pp_target.setText('ALVO: não capturado')
            self.txt_console.append('[PICKPLACE] === CICLO CONCLUÍDO ===')
            return
        name,x,y,z,tool=self.pick_cycle_steps[self.pick_cycle_index]
        self.lbl_pp_status.setText(f'SEQUÊNCIA {self.pick_cycle_index+1}/{len(self.pick_cycle_steps)}: {name}')
        cid=self.send_teach_target(x,y,z,tool,f'PICKPLACE {name}')
        if cid==-1:
            self.cancel_pick_place_cycle(f'Falha ao enviar etapa {name}.')
            return
        self.pick_cycle_waiting_id=cid

    def cancel_pick_place_cycle(self,reason='Sequência cancelada.'):
        if self.pick_cycle_active:
            self.txt_console.append(f'[PICKPLACE] CANCELADA: {reason}')
        self.pick_cycle_active=False; self.pick_cycle_steps=[]; self.pick_cycle_index=0; self.pick_cycle_waiting_id=None
        if hasattr(self,'lbl_pp_status'): self.lbl_pp_status.setText(f'SEQUÊNCIA: {reason}')

    def on_robot_state_changed(self,state):
        self.lbl_robot_state.setText(state); self.lbl_home_state.setText(f'Estado do controlador: {state}')
        colors={'READY':'#2e9f61','MOVING':'#315fe9','PAUSED':'#d39b20','HOMING':'#d39b20','NOT_HOMED':'#c47a32','STOPPED':'#c83b3b','EMERGENCY':'#c83b3b','EMERGENCY_RELEASED':'#d39b20','ERROR':'#c83b3b','CONECTADO':'#315fe9','DESCONECTADO':'#687382','RESTARTING':'#687382'}
        c=colors.get(state,'#687382'); self.lbl_robot_state.setStyleSheet(f'background:{c};color:white;font-size:16pt;font-weight:700;padding:8px 14px;border-radius:8px;')
        online=state!='DESCONECTADO'; self.lbl_serial_dot.setText('● ESP32 ONLINE' if online else '● ESP32 OFFLINE'); self.lbl_serial_dot.setStyleSheet('color:#43c47a;' if online else 'color:#c65a5a;')
        self.btn_movej.setEnabled(state=='READY'); self.btn_pause.setEnabled(state=='MOVING'); self.btn_resume.setEnabled(state=='PAUSED'); self.btn_home.setEnabled(state in ('NOT_HOMED','READY')); self.btn_stop.setEnabled(online and state not in ('EMERGENCY','EMERGENCY_RELEASED')); self.btn_reset.setEnabled(online)
        jog_ok=(state=='READY')
        if self.pick_cycle_active and state in ('STOPPED','EMERGENCY','EMERGENCY_RELEASED','NOT_HOMED','ERROR','RESTARTING','DESCONECTADO'):
            self.cancel_pick_place_cycle(f'Interrompida pelo estado {state}.')
        if hasattr(self,'jog_buttons'):
            for b in self.jog_buttons: b.setEnabled(jog_ok)
        if hasattr(self,'lbl_interlocks'):
            self.lbl_interlocks.setText(('✓ ESP32 conectado\n✓ Estado READY confirmado\n✓ JOG liberado') if jog_ok
                                        else f'{"✓" if online else "✕"} ESP32 {"conectado" if online else "desconectado"}\n✕ Estado READY não confirmado ({state})\n✕ JOG bloqueado')
        if state=='STOPPED': guide='STOPPED: inspecione fisicamente o robô e use RESET/REARMAR com confirmação supervisória.'
        elif state=='NOT_HOMED': guide='Execute HOME para referenciar o robô.'
        elif state=='READY': guide='Robô READY: comandos de movimento estão liberados.'
        elif state=='PAUSED': guide='Movimento pausado. RESUME retoma; STOP encerra e exige rearme supervisório.'
        elif state=='MOVING': guide='Movimento em execução.'
        elif state=='HOMING': guide='Homing em execução. Aguarde conclusão.'
        elif state=='EMERGENCY': guide='EMERGÊNCIA ATIVA: comandos de movimento bloqueados. Libere a botoeira antes do rearme.'
        elif state=='EMERGENCY_RELEASED': guide='EMERGÊNCIA LIBERADA: robô continua bloqueado. Inspecione a área e use RESET/REARMAR.'
        elif state=='RESTARTING': guide='Controlador reiniciando. Aguarde a ressincronização automática com a M3.9.'
        else: guide='Conecte o ESP32 #2 para iniciar.'
        self.lbl_guidance.setText(guide); self.lbl_interlocks.setText(('✓ ESP32 conectado' if online else '✕ ESP32 desconectado')+'\n'+('✓ READY confirmado' if state=='READY' else '✕ READY não confirmado')+'\n'+('✓ Movimento manual liberado' if state=='READY' else '✕ Movimento manual bloqueado'))
        if state=='READY' and self.pick_cycle_active:
            QTimer.singleShot(80,self._advance_pick_place_cycle)

    def on_command_finished(self,cid,cname):
        self.txt_console.append(f'[OK] ID:{cid:04d} {cname}')
        if cname=='MOVEJ' and self.teach_pending and cid==self.teach_pending[0]:
            _,self.teach_x,self.teach_y,self.teach_z,self.teach_tool=self.teach_pending
            self.teach_pending=None
            self.update_teach_reference_labels()
        if self.pick_cycle_active and self.pick_cycle_waiting_id==cid and cname=='MOVEJ':
            self.pick_cycle_waiting_id=None
            self.pick_cycle_index+=1
            QTimer.singleShot(80,self._advance_pick_place_cycle)
    def on_protocol_error(self,cid,err):
        self.txt_console.append(f'[ERRO] ID:{cid:04d} {err}')
        if self.teach_pending and cid==self.teach_pending[0]: self.teach_pending=None
        if self.pick_cycle_active and self.pick_cycle_waiting_id==cid:
            self.cancel_pick_place_cycle(f'Erro na etapa ativa: {err}')
        if err.startswith('EMERGENCY_STOP'):
            QMessageBox.warning(self,'Movimento interrompido','O movimento ativo foi interrompido pela emergência física.')
        elif err.startswith('EMERGENCY_ACTIVE'):
            self.serial_worker.current_state = 'EMERGENCY'
            self.serial_worker.sig_state_changed.emit('EMERGENCY')
            QMessageBox.warning(self,'Emergência ativa','RESET recusado: a emergência física ainda está acionada.')
        else:
            QMessageBox.warning(self,'Resposta do controlador',f'ID {cid:04d}\\n{err}')

    def update_main_camera(self,frame,mask):
        self.lbl_camera_dot.setText('● CÂMERA ONLINE'); self.lbl_camera_dot.setStyleSheet('color:#43c47a;')
        self.lbl_cam_main.setPixmap(QPixmap.fromImage(frame).scaled(self.lbl_cam_main.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
        if hasattr(self,'lbl_cam_teach'): self.lbl_cam_teach.setPixmap(QPixmap.fromImage(frame).scaled(self.lbl_cam_teach.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
        if hasattr(self,'lbl_hsv_mask'):
            self.lbl_hsv_mask.setPixmap(QPixmap.fromImage(mask).scaled(self.lbl_hsv_mask.size(),Qt.KeepAspectRatio,Qt.FastTransformation))
    def update_calib_camera(self,frame):
        self.last_calib_qimage=frame; self.lbl_cam_calib.setPixmap(QPixmap.fromImage(frame).scaled(self.lbl_cam_calib.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
    def on_target_detected(self,x,y,shape,angle):
        self.target_x,self.target_y,self.target_shape,self.target_angle=x,y,shape,angle; self.lbl_target_info.setText(f'TARGET VISUAL (somente monitoramento)\n{shape}\nX {x:.1f} mm   Y {y:.1f} mm\nÂngulo {angle:.1f}°')

    def on_calib_click(self,x,y):
        if len(self.calib_clicks_px)>=4 or self.last_calib_qimage is None:return
        sw,sh=self.last_calib_qimage.width(),self.last_calib_qimage.height(); lw,lh=self.lbl_cam_calib.width(),self.lbl_cam_calib.height(); scale=min(lw/sw,lh/sh); dw,dh=sw*scale,sh*scale; ox=(lw-dw)/2; oy=(lh-dh)/2
        if x<ox or x>ox+dw or y<oy or y>oy+dh:return
        px=int((x-ox)/scale); py=int((y-oy)/scale); self.calib_clicks_px.append([px,py]); self.camera_worker.set_calib_points(self.calib_clicks_px); self.txt_console.append(f'[CALIB] P{len(self.calib_clicks_px)} = ({px},{py}) px')
    def reset_calib_points(self): self.calib_clicks_px=[]; self.camera_worker.set_calib_points([]); self.lbl_calib_dot.setText('● CALIBRAÇÃO'); self.lbl_calib_dot.setStyleSheet('color:#c8a24d;')
    def calculate_homography(self):
        if len(self.calib_clicks_px)!=4: QMessageBox.warning(self,'Calibração','Marque exatamente 4 vértices no vídeo.'); return
        pc=np.array(self.calib_clicks_px,dtype=np.float32); pr=np.array([[a.value(),b.value()] for a,b in self.spn_coords],dtype=np.float32); H,_=cv2.findHomography(pc,pr)
        if H is None: QMessageBox.critical(self,'Calibração','Não foi possível calcular a homografia.'); return
        np.save(HOMOGRAPHY_FILE,H); self.camera_worker.load_homography(); self.lbl_calib_dot.setText('● CALIBRAÇÃO VÁLIDA'); self.lbl_calib_dot.setStyleSheet('color:#43c47a;'); QMessageBox.information(self,'Calibração','Homografia salva. A ROI visual passa a limitar a detecção.')

    def search_manual(self,text):
        if not text.strip(): self.manual_browser.setPlainText(self.manual_text); return
        lines=self.manual_text.splitlines(); q=text.lower(); hits=[]
        for i,line in enumerate(lines):
            if q in line.lower(): hits.extend(lines[max(0,i-2):min(len(lines),i+4)]+[''])
        self.manual_browser.setPlainText('\n'.join(hits) if hits else 'Nenhum tópico encontrado.')

    def closeEvent(self,event):
        self.cfg['camera_index']=self.camera_worker.camera_index; self.apply_hsv_controls(); self.apply_tracking_controls(); ConfigManager.save(self.cfg); self.camera_worker.stop(); self.serial_worker.stop(); event.accept()

if __name__ == '__main__':
    app=QApplication(sys.argv); app.setApplicationName('DELTA VISION CONTROL'); ide=DeltaRobotIDE(); ide.show(); sys.exit(app.exec_())
