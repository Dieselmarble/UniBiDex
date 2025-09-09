#!/usr/bin/env python3
import sys
import os
import subprocess
import re
import glob
from PySide6 import QtWidgets, QtGui, QtCore
import cv2

# Parse lsusb for USB Camera IDs
def get_usb_camera_ids():
    try:
        out = subprocess.check_output(['lsusb'], text=True)
    except Exception as e:
        print(f'Error running lsusb: {e}')
        return set()
    ids = set()
    for line in out.splitlines():
        if 'usb camera' in line.lower():
            m = re.search(r'ID ([0-9A-Fa-f]{4}:[0-9A-Fa-f]{4})', line)
            if m:
                ids.add(m.group(1).lower())
    return ids

# Get vendor:product for a /dev/videoX by walking sysfs
def get_device_id(dev_path):
    basename = os.path.basename(dev_path)
    sysdev = os.path.join('/sys/class/video4linux', basename, 'device')
    try:
        path = os.path.realpath(sysdev)
    except:
        return None
    while True:
        vfile = os.path.join(path, 'idVendor')
        pfile = os.path.join(path, 'idProduct')
        if os.path.exists(vfile) and os.path.exists(pfile):
            try:
                vendor = open(vfile).read().strip().lower()
                product = open(pfile).read().strip().lower()
                return f"{vendor}:{product}"
            except:
                return None
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None

class CameraAssigner(QtWidgets.QWidget):
    def __init__(self, devices, fps=30):
        super().__init__()
        self.devices = devices
        self.fps = fps
        self.roles = ['camera_left_wrist', 'camera_right_wrist',
                      'camera_front', 'camera_left', 'camera_right']
        self.mapping = {}
        self.index = 0

        # UI setup
        self.setWindowTitle('USB Camera Role Selector')
        self.video_label = QtWidgets.QLabel()
        self.video_label.setFixedSize(640, 480)
        btn_layout = QtWidgets.QHBoxLayout()
        for role in self.roles + ['skip']:
            btn = QtWidgets.QPushButton(role)
            btn.clicked.connect(lambda checked, r=role: self.select(r))
            btn_layout.addWidget(btn)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.video_label)
        layout.addLayout(btn_layout)

        self.cap = None
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.show_frame)
        self.next_camera()

    def next_camera(self):
        # Release previous capture
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.index >= len(self.devices):
            self.finish()
            return
        dev = self.devices[self.index]
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.open(dev, cv2.CAP_ANY)
        if self.cap.isOpened():
            self.timer.start(int(1000/self.fps))
        else:
            print(f'Cannot open {dev}, skipping')
            self.index += 1
            self.next_camera()

    def show_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        img = QtGui.QImage(rgb.data, w, h, 3*w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(img)
        self.video_label.setPixmap(pix)

    def select(self, role):
        dev = self.devices[self.index]
        if role != 'skip':
            if role in self.mapping:
                QtWidgets.QMessageBox.warning(self, 'Warn', f'{role} already assigned')
                return
            self.mapping[role] = dev
        self.index += 1
        self.next_camera()

    def finish(self):
        self.timer.stop()
        if self.cap and self.cap.isOpened():
            self.cap.release()
        summary = '\n'.join(f'{r}: {d}' for r, d in self.mapping.items())
        QtWidgets.QMessageBox.information(self, 'Done', f'Assigned:\n{summary}')
        print('Final mapping:', self.mapping)

        # Generate udev rules using ENV{ID_PATH}
        rules = []
        for role, dev in self.mapping.items():
            # Query ID_PATH via udevadm
            try:
                info = subprocess.check_output(
                    ['udevadm', 'info', '-q', 'property', '-n', dev], text=True
                )
                path_line = next(
                    (l for l in info.splitlines() if l.startswith('ID_PATH=')),
                    None
                )
                if path_line:
                    id_path = path_line.split('=',1)[1]
                    rule = (
                        f'SUBSYSTEM=="video4linux", ENV{{ID_PATH}}=="{id_path}", '
                        f'SYMLINK+="{role}"'
                    )
                    rules.append(rule)
                else:
                    raise ValueError('No ID_PATH')
            except Exception as e:
                # Fallback comment for unmatched device
                did = get_device_id(dev)
                vendor, product = did.split(':') if did else ('', '')
                rule = (
                    f'# {role}: failed to get ID_PATH, use VID:PID instead\n'
                    f'SUBSYSTEM=="video4linux", ATTRS{{idVendor}}=="{vendor}", '
                    f'ATTRS{{idProduct}}=="{product}", SYMLINK+="{role}"'
                )
                rules.append(rule)

        if rules:
            out_file = 'camera_roles.rules'
            try:
                with open(out_file, 'w') as f:
                    f.write('\n'.join(rules) + '\n')
                print(f'Udev rules written to {out_file}. Copy it to /etc/udev/rules.d/ and reload rules.')
            except Exception as e:
                print(f'Failed to write udev rules: {e}')

        QtWidgets.QApplication.quit()


def main():
    usb_ids = get_usb_camera_ids()
    if not usb_ids:
        print('No USB Camera found in lsusb')
        return
    devices = []
    # new: Only take from each physical port“index0”那一路
    for entry in sorted(glob.glob('/dev/v4l/by-path/*-video-index0')):
        # Real device node /dev/videoX
        dev = os.path.realpath(entry)
        did = get_device_id(dev)
        if did and did in usb_ids:
            devices.append(dev)
    if not devices:
        print('No matching /dev/video* devices')
        return
    app = QtWidgets.QApplication(sys.argv)
    w = CameraAssigner(devices)
    w.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

