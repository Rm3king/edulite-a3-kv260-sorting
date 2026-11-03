"""Read-only KV260 wrist-camera focus window; --run explicitly connects."""
import argparse
import http.client
import json
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit
import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal, QThread
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget


class Reader(QThread):
    frame = pyqtSignal(bytes, dict)
    failed = pyqtSignal(str)

    def __init__(self, url):
        super().__init__(); self.url=urlsplit(url); self.stop=threading.Event()

    def run(self):
        connection=None
        while not self.stop.is_set():
            started=time.monotonic()
            try:
                if connection is None:
                    connection=http.client.HTTPConnection(self.url.hostname,self.url.port,timeout=1.)
                connection.request('GET','/frame.jpg')
                response=connection.getresponse();data=response.read();received=time.monotonic()
                if response.status!=200:raise RuntimeError(f'Camera HTTP {response.status}: {data[:160]!r}')
                info=dict(sequence=int(response.getheader('X-Frame-Sequence')),
                          camera_session=response.getheader('X-Camera-Session'),
                          board_received_monotonic_ns=int(response.getheader('X-Board-Received-Monotonic-Ns')),
                          board_frame_age_s=float(response.getheader('X-Board-Frame-Age-S')),
                          host_received_monotonic_s=received,request_s=received-started,
                          clock_synchronized=False,robot_commands_sent=0)
                self.frame.emit(data,info)
            except Exception as error:
                if connection:connection.close()
                connection=None;self.failed.emit(str(error));self.stop.wait(.3)
            self.stop.wait(max(0,.05-(time.monotonic()-started)))
        if connection:connection.close()


class Preview(QWidget):
    def __init__(self,url,out):
        super().__init__();self.out=out;self.last_saved=0
        self.setWindowTitle('腕部相机 · KV260 USB · 调焦预览（不控制机械臂）')
        self.resize(980,720);layout=QVBoxLayout(self)
        title=QLabel('保持机械臂不动，缓慢旋转镜头调焦；先看清螺丝边缘和螺纹。')
        title.setWordWrap(True);layout.addWidget(title)
        self.picture=QLabel('等待新画面…');self.picture.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.picture.setMinimumSize(640,400);layout.addWidget(self.picture,1)
        self.status=QLabel('连接中');self.status.setWordWrap(True);layout.addWidget(self.status)
        layout.addWidget(QLabel('清晰度分数仅用于同一场景调焦比较，不代表毫米精度；尚未标定，不发送抓取纠偏。'))
        self.reader=Reader(url);self.reader.frame.connect(self.update_frame);self.reader.failed.connect(self.error);self.reader.start()

    def update_frame(self,data,info):
        image=QImage.fromData(data)
        if image.isNull():self.error('JPEG无法解码');return
        gray=image.convertToFormat(QImage.Format.Format_Grayscale8)
        ptr=gray.constBits();ptr.setsize(gray.sizeInBytes())
        a=np.frombuffer(ptr,np.uint8).reshape(gray.height(),gray.bytesPerLine())[:,:gray.width()].astype(float)
        h,w=a.shape;roi=a[h//4:3*h//4,w//4:3*w//4]
        lap=roi[:-2,1:-1]+roi[2:,1:-1]+roi[1:-1,:-2]+roi[1:-1,2:]-4*roi[1:-1,1:-1]
        focus=float(lap.var());info.update(focus_proxy_center_laplacian_variance=focus,width=image.width(),height=image.height())
        self.picture.setPixmap(QPixmap.fromImage(image).scaled(self.picture.size(),Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation))
        self.status.setText(f"{image.width()}×{image.height()}  帧 {info['sequence']}  中央清晰度 {focus:.1f}\n板端帧龄 {info['board_frame_age_s']*1000:.0f} ms  请求耗时 {info['request_s']*1000:.0f} ms（两端时钟未同步）")
        now=time.monotonic()
        if now-self.last_saved>1:
            self.last_saved=now;(self.out/'latest.jpg').write_bytes(data)
            tmp=self.out/'latest.tmp';tmp.write_text(json.dumps(info,ensure_ascii=False,indent=2));tmp.replace(self.out/'latest.json')

    def error(self,message):
        self.picture.clear();self.picture.setText('没有可用的新画面')
        self.status.setText(message)

    def closeEvent(self,event):
        self.reader.stop.set();self.reader.wait(2000);event.accept()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:18766')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--run',action='store_true')
    args=parser.parse_args()
    if not args.run:
        print(json.dumps(dict(status='DRY_RUN',camera_connected=False,motor_commands_sent=0)));return
    args.out.mkdir(parents=True,exist_ok=True)
    app=QApplication([]);window=Preview(args.url,args.out);window.show();app.exec()


if __name__=='__main__':main()
