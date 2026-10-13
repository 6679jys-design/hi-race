"""D435 raw depth ROI baseline. Q: quit. Logs camera Z, not radial range."""
import csv
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    out = Path(__file__).parent / 'logs' / datetime.now().strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_record_to_file(str(out / 'raw.db3'))
    started = False
    try:
        print('Starting D435 RGB + Depth...', flush=True)
        profile = pipeline.start(config)
        started = True
        sensor = profile.get_device().first_depth_sensor()
        scale = sensor.get_depth_scale()
        print('Depth scale (m/unit):', scale)
        print('Center the target inside the rectangle. Q quits.')
        print('Output:', out)
        with (out / 'roi.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['host_monotonic_s', 'depth_frame', 'depth_timestamp_ms',
                             'timestamp_domain', 'median_z_m', 'valid_ratio', 'mad_m'])
            while True:
                frames = pipeline.wait_for_frames(5000)
                depth, color = frames.get_depth_frame(), frames.get_color_frame()
                if not depth or not color:
                    continue
                # Native depth coordinates; no RGB alignment or filtering in this baseline.
                raw = np.asanyarray(depth.get_data())
                h, w = raw.shape
                x0, x1, y0, y1 = w//2-40, w//2+40, h//2-40, h//2+40
                roi = raw[y0:y1, x0:x1]
                values = roi[roi > 0].astype(np.float64) * scale
                ratio = values.size / roi.size
                median = float(np.median(values)) if values.size else float('nan')
                mad = float(np.median(np.abs(values-median))) if values.size else float('nan')
                writer.writerow([time.monotonic(), depth.get_frame_number(),
                                 depth.get_timestamp(), str(depth.get_frame_timestamp_domain()),
                                 median, ratio, mad])
                view = cv2.applyColorMap(cv2.convertScaleAbs(raw * scale, alpha=255/4), cv2.COLORMAP_JET)
                view[raw == 0] = 0
                cv2.rectangle(view, (x0, y0), (x1, y1), (255, 255, 255), 2)
                cv2.putText(view, f'Z={median:.3f} m valid={ratio:.0%}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
                cv2.imshow('D435 native depth ROI', view)
                cv2.imshow('D435 RGB reference', np.asanyarray(color.get_data()))
                if cv2.waitKey(1) & 0xff == ord('q'):
                    break
    finally:
        if started:
            pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
