#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import torch
import numpy as np
from facenet_pytorch import InceptionResnetV1
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')

        # 初始化 OpenCV 與 ROS 相關物件
        self.bridge = CvBridge()

        # 訂閱攝影機影像
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        # 發佈辨識結果
        self.result_pub = self.create_publisher(String, '/face_recognition/result', 10)

        # 清除 GPU 記憶體
        torch.cuda.empty_cache()

        # 載入 YOLOv8/YOLOv10 人臉偵測模型
        self.face_detector = YOLO('/home/jason9308/robot_ws/src/face_recognition/best.pt') # 你可以換成 yolov10-face.pt
        
        # 載入 FaceNet (人臉辨識模型)
        self.face_recognizer = InceptionResnetV1(pretrained='vggface2').eval()

        # 設定參考影像 (用來進行身份比對)
        self.reference_embedding = self.load_reference_image('/home/jason9308/robot_ws/src/face_recognition/me.jpeg')


        # 設定辨識參數
        self.threshold = 0.75  # 相似度閾值
        self.pass_count = 0
        self.pass_required = 70  # 需要累積 20 次高於閾值才通過

    def load_reference_image(self, img_path):
        """載入參考影像，提取特徵向量"""
        reference_image = cv2.imread(img_path)
        if reference_image is None:
            self.get_logger().error("無法讀取參考影像！")
            return None

        # 偵測參考影像中的人臉
        results = self.face_detector(reference_image)
        if results and results[0].boxes:
            x1, y1, x2, y2 = map(int, results[0].boxes[0].xyxy[0].tolist())
            reference_face = reference_image[y1:y2, x1:x2]
            return self.get_embedding(reference_face)
        else:
            self.get_logger().error("未偵測到參考人臉！")
            return None

    def process_face(self, face_img):
        """影像前處理 (轉換尺寸、正規化、轉 Tensor)"""
        face_img = cv2.resize(face_img, (160, 160))
        face_img = np.transpose(face_img, (2, 0, 1))  # (H, W, C) -> (C, H, W)
        face_img = face_img / 255.0  # 正規化
        face_img = torch.tensor(face_img).float().unsqueeze(0)  # 轉為 Tensor
        return face_img

    def get_embedding(self, face_img):
        """計算人臉特徵向量"""
        face_tensor = self.process_face(face_img)
        with torch.no_grad():
            embedding = self.face_recognizer(face_tensor)
        return embedding.squeeze().detach().numpy()

    def image_callback(self, msg):
        """當攝影機影像進來時，進行人臉偵測與辨識"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"影像轉換失敗: {e}")
            return

        # YOLO 偵測人臉
        results = self.face_detector(frame)

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            confidence = box.conf[0]

            # 擷取偵測到的人臉
            face_img = frame[y1:y2, x1:x2]

            # 計算特徵向量
            detected_embedding = self.get_embedding(face_img)

            # 計算餘弦相似度
            if self.reference_embedding is not None:
                cosine_similarity = np.dot(self.reference_embedding, detected_embedding) / (
                    np.linalg.norm(self.reference_embedding) * np.linalg.norm(detected_embedding)
                )
            else:
                cosine_similarity = 0.0  # 若無參考影像，則不計算

            # 更新辨識累積計數
            if cosine_similarity > self.threshold:
                self.pass_count += 1
            else:
                self.pass_count = 0  # 若有一次不合格，則重新計數

            # 發佈辨識結果
            # result_msg = String()
            # result_msg.data = f"Similarity: {cosine_similarity:.2f}, Confidence: {confidence:.2f}, Pass Count: {self.pass_count}/{self.pass_required}"
            # self.result_pub.publish(result_msg)

            # 在畫面上繪製框線與相似度
            label = f"Sim: {cosine_similarity:.2f} | Conf: {confidence:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 若達到通過標準，發佈成功訊息並關閉節點
            if self.pass_count >= self.pass_required:
                success_msg = String()
                success_msg.data = "身份確認成功"
                self.result_pub.publish(success_msg)
                self.get_logger().info("身份確認成功！即將關閉節點...")

                # 顯示成功訊息並延遲 3 秒
                cv2.putText(frame, "Access Granted!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Face Recognition", frame)
                cv2.waitKey(3000)  # 顯示 3 秒後再關閉節點
                
                cv2.destroyAllWindows()  # 關閉 OpenCV 視窗
                self.image_sub = None  # 解除訂閱
                self.result_pub = None  # 解除 Publisher
                self.destroy_node()
                exit(0)  # !!! 請注意，這裡是強制結束程式 !!!
                return

        # 顯示畫面
        cv2.imshow("Face Recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("手動結束辨識")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()

    if rclpy.ok():
        rclpy.spin(node)

    # node.destroy_node()
    # rclpy.shutdown()
    return


if __name__ == "__main__":
    main()
