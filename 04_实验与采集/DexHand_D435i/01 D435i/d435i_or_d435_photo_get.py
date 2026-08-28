import pyrealsense2 as rs
import numpy as np
import cv2
import keyboard  # 这个在windows上用比较方便， 在linux上会因为权限问题难以获得全局键盘状态监控
import time
import threading
import os


class D435Camera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.color_intrin = None
        self.depth_intrin = None
        self.aligned_depth_frame = None
        self.apple_pose = None
        # 标定板照片与相机内参保存目录（眼在手上标定使用）
        self.save_dir = r'C:\Users\sxy18\Desktop\记录留痕\DexHand_D435i\01 D435i\img_cam_calibration'
        self.frame_count = 0  # 用于计算帧数的变量
        self.start_time = time.time()
        self.font = cv2.FONT_HERSHEY_SIMPLEX  # 定义字体

    def initialize(self):
        config = rs.config()
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 15)
        pipe_profile = self.pipeline.start(config)

        # 自动获取彩色流相机内参（无需硬编码，运行时 get_aligned_images 也会持续更新）
        color_profile = pipe_profile.get_stream(rs.stream.color)
        self.color_intrin = color_profile.as_video_stream_profile().intrinsics

    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        self.aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        self.color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics

        img_color = np.asanyarray(aligned_color_frame.get_data())
        img_depth = np.asanyarray(self.aligned_depth_frame.get_data())

        return img_color, img_depth

    def get_3d_camera_coordinate(self, depth_pixel):
        x = depth_pixel[0]
        y = depth_pixel[1]
        depth = self.aligned_depth_frame.get_distance(x, y)
        camera_coordinate = rs.rs2_deproject_pixel_to_point(self.depth_intrin, depth_pixel, depth)
        return depth, camera_coordinate

    def apple_event(self, event):
        if event.name == 'a':
            self.get_apple()
            time.sleep(1)

    def run(self):
        self.initialize()
        global img_color, img_depth, frame, centers
        while True:
            img_color, img_depth = self.get_aligned_images()
            # self.frame_count += 1  # 更新帧数
            self.frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            if elapsed_time >= 1:  # 计算每秒的帧率
                fps = self.frame_count / elapsed_time
                self.frame_count = 0
                self.start_time = current_time
                # 在帧上绘制帧数
            # cv2.putText(frame, f"FPS: {fps:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            # cv2.putText(frame, f"Frame: {self.frame_count}", (10, 30), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # # 显示帧
            # cv2.imshow('Video', frame)

            cv2.imshow("frame", img_color)

            if cv2.waitKey(1) == ord('q'):
                break

        self.pipeline.stop()
        cv2.destroyAllWindows()


    def take_photo(self, interval=1):

        self.initialize()
        os.makedirs(self.save_dir, exist_ok=True)
        i = 1
        while True:
            img_color, img_depth = self.get_aligned_images()

            cv2.imshow("frame", img_color)
            k = cv2.waitKey(interval)
            if k == ord('s'):
                img_path = os.path.join(self.save_dir, f"{i}.jpg")
                cv2.imwrite(img_path, img_color)
                print(f"照片保存成功: {img_path}")
                i += 1
            elif k == ord("q"):
                break

        # cap.release()
        self.pipeline.stop()
        cv2.destroyAllWindows()


# -------------------------------------------------------------10.20
def camera_run():
    d435_camera.run()


def crl_flag_test():
    d435_camera.take_photo(1)

if __name__ == "__main__":
    d435_camera = D435Camera()
    print("按下s保存图片，按下Q退出程序")

    t1 = threading.Thread(target=crl_flag_test)
    t1.start()


