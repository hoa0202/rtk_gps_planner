import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import String
import time

class YOLOv8Processor(Node):
    def __init__(self):
        super().__init__('yolov8_processor')
        
        # ROS2 구독자 설정: RGB 이미지, 깊이 이미지, 카메라 정보, cmd_vel을 수신
        # queue_size=1로 최신 프레임만 처리 (과부하 방지)
        self.image_subscription = self.create_subscription(
            Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 1)
        self.depth_subscription = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 1)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/zed/zed_node/depth/camera_info', self.camera_info_callback, 1)
        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ROS2 퍼블리셔 설정: 상태 정보, 이미지, 장애물 감지 시 속도 명령을 발행
        self.chatter_publisher = self.create_publisher(String, '/chatter', 10)
        self.image_publisher = self.create_publisher(CompressedImage, '/processed_image/compressed2', 10)
        self.cmd_vel_main_publisher = self.create_publisher(Twist, '/cmd_vel_main', 10)

        # 기타 변수 초기화
        self.bridge = CvBridge()  # cv_bridge 설정 (ROS 이미지 <-> OpenCV 이미지 변환)
        self.model = YOLO('./yolov8m.pt')  # YOLOv8 모델 로드
        self.depth_image = None  # 깊이 이미지 저장 변수
        self.camera_info = None  # 카메라 정보 저장 변수
        self.image_msg = None  # RGB 이미지 메시지 저장 변수
        self.distance_threshold = 4.0  # 장애물 감지 거리 임계값 (미터)
        self.detection_threshold = 1.0  # 감지 지속 시간 임계값 (초)
        self.last_detection_change_time = None  # 마지막 감지 상태 변경 시간
        self.previous_detection = 'no_detect'  # 이전 감지 상태
        self.current_cmd_vel = None  # 현재 속도 명령 저장 변수
        
        # 추론 주기 제한 (과부하 방지)
        self.last_inference_time = 0
        self.inference_interval = 0.2  # 0.2초마다 추론 (5fps)
        self.debug_mode = False  # True면 cv2.imshow 활성화

    def camera_info_callback(self, msg):
        # 카메라 정보를 저장하는 콜백 함수
        self.camera_info = msg

    def image_callback(self, msg):
        # RGB 이미지 수신 콜백 함수
        self.image_msg = msg
        
        # 추론 주기 제한 (과부하 방지)
        current_time = time.time()
        if current_time - self.last_inference_time < self.inference_interval:
            return  # 스킵
        self.last_inference_time = current_time
        
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # ROS 이미지 -> OpenCV 이미지 변환
            self.process_image(image)  # 이미지 처리 함수 호출
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")  # 변환 실패 시 오류 출력

    def depth_callback(self, msg):
        # 깊이 이미지 수신 콜백 함수
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  # ROS -> OpenCV 변환
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {str(e)}")  # 변환 실패 시 오류 출력

    def cmd_vel_callback(self, msg):
        # cmd_vel 명령 수신 콜백 함수
        self.current_cmd_vel = msg  # 현재 cmd_vel 명령을 저장
        if self.previous_detection == 'no_detect':  # 장애물이 감지되지 않았을 때만 발행
            self.cmd_vel_main_publisher.publish(msg)

    def process_image(self, image):
        # 이미지 처리 함수 (YOLOv8 모델을 사용해 객체 탐지 수행)
        results = self.model(image)

        highest_confidence = 0.65  # 탐지 임계값
        best_bbox = None  # 가장 신뢰도가 높은 바운딩 박스 초기화
        min_z_distance = float('inf')  # 장애물의 최단 거리 초기화

        # 탐지 결과를 반복하여 확인
        for result in results:
            for box in result.boxes:
                cls_index = box.cls  # 클래스 인덱스
                conf = box.conf.item()  # 신뢰도
                cls_name = self.model.names[int(cls_index)]  # 클래스 이름

                if cls_name == 'person' and conf > highest_confidence:  # 'person' 클래스와 신뢰도 조건 확인
                    highest_confidence = conf
                    best_bbox = box.xyxy.cpu().numpy()  # 바운딩 박스 좌표 업데이트

        # 장애물 위치와 카메라 정보가 모두 있을 때 거리 계산
        if best_bbox is not None and self.depth_image is not None and self.camera_info is not None:
            x1, y1, x2, y2 = map(int, best_bbox[0])  # 바운딩 박스 좌표
            fx = self.camera_info.k[0]  # 카메라의 초점 거리 x
            fy = self.camera_info.k[4]  # 카메라의 초점 거리 y
            cx = self.camera_info.k[2]  # 카메라 중심 x 좌표
            cy = self.camera_info.k[5]  # 카메라 중심 y 좌표

            num_points = 15  # 가로 방향으로 측정할 점의 수
            x_coords = np.linspace(x1, x2, num=num_points, dtype=int)  # 측정할 x 좌표들

            for x in x_coords:
                y = (y1 + y2) // 2  # 바운딩 박스의 수직 중앙선
                if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                    z_distance = float(self.depth_image[y, x])  # 깊이 값 (전방 거리)
                    if 0 < z_distance < min_z_distance:
                        min_z_distance = z_distance  # 가장 작은 z 거리 업데이트

                    # 이미지에 점과 거리 표시
                    cv2.circle(image, (x, y), 3, (0, 0, 255), -1)
                    cv2.putText(image, f"{z_distance:.2f}", (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

            if min_z_distance < float('inf'):
                # 좌우 거리 계산
                x_distance = float((x - cx) * min_z_distance / fx)

                # 거리 정보를 이미지에 표시
                cv2.putText(image, f"X (left/right): {x_distance:.2f} m", (x2 + 10, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image, f"Z (forward/backward): {min_z_distance:.2f} m", (x2 + 10, y1 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.get_logger().info(f"Distance to person: X: {x_distance:.2f} meters, Z: {min_z_distance:.2f} meters")

            # 바운딩 박스 및 중심 표시
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            best_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            cv2.circle(image, best_center, 5, (0, 0, 255), -1)

        # Z 거리가 임계값 이하인 경우 'detect' 상태 설정
        current_detection = 'detect' if min_z_distance <= self.distance_threshold else 'no_detect'
        current_time = time.time()

        # 감지 상태가 변경되면 상태 업데이트 및 타이머 초기화
        if current_detection != self.previous_detection:
            self.last_detection_change_time = current_time
            self.previous_detection = current_detection
        elif self.last_detection_change_time is not None and (current_time - self.last_detection_change_time) >= self.detection_threshold:
            # 장애물이 계속 감지되는 경우 장애물 경고 발행
            if current_detection == 'detect':
                self.publish_obstacle_warning()
                self.last_detection_change_time = current_time  # 메시지를 발행한 후 타이머 초기화
            # 감지되지 않으면 현재 속도 명령 발행
            elif current_detection == 'no_detect' and self.current_cmd_vel is not None:
                self.cmd_vel_main_publisher.publish(self.current_cmd_vel)

        # 처리된 이미지를 압축하여 발행
        try:
            compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            self.image_publisher.publish(compressed_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish compressed image: {str(e)}")

        # 감지된 객체와 정보를 화면에 표시 (디버그 모드일 때만)
        if self.debug_mode:
            cv2.imshow('YOLOv8 Detection', image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
                cv2.destroyAllWindows()

    def publish_obstacle_warning(self):
        # cmd_vel_main의 모든 값을 0으로 만들어 정지 명령 발행
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_main_publisher.publish(stop_msg)

        # 장애물 감지 상태를 'OBSTACLE'로 발행
        msg = String()
        msg.data = "OBSTACLE"
        self.chatter_publisher.publish(msg)
        self.get_logger().info("Published detection status: OBSTACLE")

def main(args=None):
    # ROS2 노드 초기화 및 실행
    rclpy.init(args=args)
    yolov8_processor = YOLOv8Processor()
    try:
        rclpy.spin(yolov8_processor)
    except KeyboardInterrupt:
        pass
    yolov8_processor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
