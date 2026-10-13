"""D435 + YOLO person-aware motion decision demo. No motor commands.

Run: python person_control.py --seconds 60
NORMAL means no person restriction in the image ROI, not a safe-to-drive verdict.
"""
import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from PySide6.QtCore import QThread, Signal, QTimer, Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton


class Decision:
    """Immediate restriction; release only after continuously milder evidence."""
    ranks = {'NORMAL': 0, 'SLOW': 1, 'STOP': 2, 'UNKNOWN': 3}

    def __init__(self, release_seconds=1.0):
        self.state = 'UNKNOWN'
        self.pending = None
        self.since = 0.0
        self.release_seconds = release_seconds

    def update(self, candidate, now):
        if self.ranks[candidate] >= self.ranks[self.state]:
            self.state = candidate
            self.pending = None
        else:
            if candidate != self.pending:
                self.pending, self.since = candidate, now
            elif now - self.since >= self.release_seconds:
                self.state, self.pending = candidate, None
        return self.state


def person_depth(depth, bbox, scale):
    """Central bbox patch, raw nonzero depth median; thresholds are demo choices."""
    h, w = depth.shape
    x1, y1, x2, y2 = bbox
    cx, cy = (x1+x2)/2, (y1+y2)/2
    rx, ry = max(2, (x2-x1)*0.15), max(2, (y2-y1)*0.15)
    patch = depth[max(0, int(cy-ry)):min(h, int(cy+ry)),
                  max(0, int(cx-rx)):min(w, int(cx+rx))]
    if not patch.size:
        return None, 0.0
    values = patch.astype(np.float64) * scale
    valid = values[np.isfinite(values) & (values >= 0.3) & (values <= 3.0)]
    ratio = valid.size / patch.size
    if valid.size < 20 or ratio < 0.3:
        return None, ratio
    return float(np.median(valid)), ratio


class CameraThread(QThread):
    frame_ready = Signal(object, object)
    error = Signal(str)

    def __init__(self, seconds):
        super().__init__()
        self.seconds = seconds
        self.output = Path(__file__).parent / 'logs' / datetime.now().strftime('control_%Y%m%d_%H%M%S_%f')

    def run(self):
        pipe, started = rs.pipeline(), False
        try:
            model = YOLO('yolov8n.pt')
            if self.isInterruptionRequested():
                return
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            profile = pipe.start(config)
            started = True
            scale = profile.get_device().first_depth_sensor().get_depth_scale()
            align, controller = rs.align(rs.stream.color), Decision()
            self.output.mkdir(parents=True)
            start = time.monotonic()
            with (self.output / 'decisions.csv').open('w', newline='', encoding='utf-8-sig') as f:
                log = csv.writer(f)
                log.writerow(['elapsed_s', 'capture_timestamp_ms', 'people', 'cars',
                              'front_people', 'nearest_person_z_m', 'candidate', 'state'])
                while not self.isInterruptionRequested() and time.monotonic()-start < self.seconds:
                    # Poll instead of an unbounded wait; GUI can request interruption.
                    frames = pipe.poll_for_frames()
                    if not frames:
                        self.msleep(10)
                        continue
                    frames = align.process(frames)
                    df, cf = frames.get_depth_frame(), frames.get_color_frame()
                    if not df or not cf:
                        continue
                    image = np.asanyarray(cf.get_data()).copy()
                    depth = np.asanyarray(df.get_data())
                    h, w = depth.shape
                    left, right = int(w*.3), int(w*.7)
                    # Image ROI is only a proxy, not the robot's calibrated swept path.
                    results = model(image, classes=[0, 2], conf=.4, verbose=False)
                    cv2.rectangle(image, (left, 0), (right, h-1), (0, 255, 255), 2)
                    people = cars = front = 0
                    distances, invalid_front = [], False
                    for result in results:
                        for box in result.boxes:
                            cls = int(box.cls.item())
                            bbox = tuple(map(int, box.xyxy[0].tolist()))
                            x1, y1, x2, y2 = bbox
                            z, ratio = person_depth(depth, bbox, scale)
                            is_front = cls == 0 and x2 > left and x1 < right
                            if cls == 0:
                                people += 1
                            else:
                                cars += 1
                            if is_front:
                                front += 1
                                if z is None:
                                    invalid_front = True
                                else:
                                    distances.append(z)
                            text = ('person' if cls == 0 else 'car') + (f' {z:.2f}m' if z is not None else ' depth?')
                            color = (0, 255, 0) if cls == 0 else (255, 128, 0)
                            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(image, text, (max(0, x1), max(18, y1-8)), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 2)
                    nearest = min(distances) if distances else None
                    if invalid_front:
                        candidate = 'UNKNOWN'
                    elif nearest is not None and nearest <= 1.0:
                        candidate = 'STOP'
                    elif nearest is not None and nearest <= 1.5:
                        candidate = 'SLOW'
                    else:
                        candidate = 'NORMAL'
                    now = time.monotonic()
                    state = controller.update(candidate, now)
                    data = dict(state=state, candidate=candidate, people=people, cars=cars,
                                front=front, nearest=nearest, elapsed=now-start)
                    log.writerow([now-start, cf.get_timestamp(), people, cars, front,
                                  '' if nearest is None else nearest, candidate, state])
                    f.flush()
                    self.frame_ready.emit(image, data)
        except Exception as exc:
            self.error.emit(f'{type(exc).__name__}: {exc}')
        finally:
            if started:
                try:
                    pipe.stop()
                except Exception as exc:
                    self.error.emit(f'Camera cleanup: {exc}')


class MainWindow(QMainWindow):
    def __init__(self, seconds):
        super().__init__()
        self.setWindowTitle('D435 person decision demo - NO MOTOR OUTPUT')
        widget, layout = QWidget(), QVBoxLayout()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        self.note = QLabel('화면 실습용: 전방 중앙 40%, STOP ≤1m, SLOW ≤1.5m. 실제 주행 안전 판단 아님.')
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        self.status = QLabel('UNKNOWN | 모델·카메라 준비 중…')
        layout.addWidget(self.status)
        self.image = QLabel()
        self.image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.image)
        self.stop_button = QPushButton('측정 종료')
        self.stop_button.clicked.connect(self.stop)
        layout.addWidget(self.stop_button)
        self.thread = CameraThread(seconds)
        self.thread.frame_ready.connect(self.update_frame)
        self.thread.error.connect(self.show_error)
        self.thread.finished.connect(self.completed)
        self.last_received = time.monotonic()
        self.closing = False
        self.failed = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_stale)
        self.timer.start(200)
        self.thread.start()

    def update_frame(self, frame, data):
        if self.thread.isInterruptionRequested():
            return
        self.last_received = time.monotonic()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        img = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format.Format_RGB888).copy()
        self.image.setPixmap(QPixmap.fromImage(img))
        z = '미확인' if data['nearest'] is None else f"{data['nearest']:.3f}m"
        self.status.setText(f"{data['state']} | 사람 {data['people']} / 차 {data['cars']} / 전방 사람 {data['front']} | 거리 {z} | {data['elapsed']:.1f}s")

    def check_stale(self):
        if self.thread.isRunning() and not self.thread.isInterruptionRequested() and not self.failed and time.monotonic()-self.last_received > 2:
            self.status.setText('UNKNOWN | 2초 이상 새 결과 없음: 초기화·느린 추론·센서 중단 확인')

    def show_error(self, message):
        self.failed = True
        self.status.setText('UNKNOWN | ' + message)

    def stop(self):
        self.thread.requestInterruption()
        self.stop_button.setEnabled(False)
        self.status.setText('종료 요청됨: 진행 중인 처리와 카메라 정리를 기다리는 중…')

    def completed(self):
        self.timer.stop()
        self.stop_button.setEnabled(False)
        if not self.failed:
            self.status.setText(f'측정 종료 | CSV: {self.thread.output / "decisions.csv"}')
        if self.closing:
            self.close()

    def closeEvent(self, event):
        if self.thread.isRunning():
            self.closing = True
            self.stop()
            event.ignore()  # Never block the GUI with QThread.wait().
        else:
            event.accept()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60, help='Capture duration after camera startup')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    app = QApplication(sys.argv[:1])
    window = MainWindow(args.seconds)
    window.show()
    sys.exit(app.exec())
