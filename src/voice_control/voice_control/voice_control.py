#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import speech_recognition as sr
import openai
import configparser
import json
import re

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')

        # 讀取 OpenAI API 金鑰
        config = configparser.ConfigParser()
        config.read('/home/jason9308/robot_ws/src/voice_control/config.ini')
        self.OPENAI_API_KEY = config['API']['api_key']

        # 初始化 OpenAI 客戶端
        self.client = openai.OpenAI(api_key=self.OPENAI_API_KEY)

        # 語音辨識
        self.recognizer = sr.Recognizer()

        # 開始語音處理
        self.listen_and_process()

    def listen_and_process(self):
        while True:  # 讓程式一直執行，直到成功解析 JSON
            with sr.Microphone() as source:
                self.get_logger().info("請說話...")
                self.recognizer.adjust_for_ambient_noise(source)
                audio_data = self.recognizer.listen(source)

                try:
                    # 語音轉文字
                    text = self.recognizer.recognize_google(audio_data, language="zh-TW")
                    self.get_logger().info(f"你說了: {text}")

                    # 呼叫 OpenAI 產生 JSON
                    response = self.generate_json(text)
                    self.get_logger().info(f"生成的 JSON: {response}")

                    # 嘗試解析並儲存 JSON
                    if self.extract_and_dump_json(response, "/home/jason9308/robot_ws/src/voice_control/output.json"):
                        self.get_logger().info("JSON 存檔成功，關閉節點")
                        self.destroy_node()  # 關閉節點
                        rclpy.shutdown()
                        break  # 成功後離開 while 迴圈
                    else:
                        self.get_logger().error("JSON 無效，請重新輸入")

                except sr.UnknownValueError:
                    self.get_logger().error("無法識別語音，請再試一次")
                except sr.RequestError as e:
                    self.get_logger().error(f"語音請求失敗: {e}")


    def generate_json(self, instruction):
        """將語音指令轉換成 JSON"""
        prompt = f"""instruction : {instruction}

        Please understand the equipment and the order in this instruction, and convert them into the following JSON schema:

        [
            {{
                "equipment": "<description of equipment>",
                "order": "<step-by-step order>"
            }}
        ]

        Please return this in valid JSON format.
        """

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are an AI assistant that converts instructions into structured JSON format."},
                {"role": "user", "content": prompt}
            ]
        )

        return response.choices[0].message.content  # 取得 LLM 產生的 JSON

    def extract_and_dump_json(self, sample_text, output_file_path):
        """解析並存儲 JSON"""
        try:
            parsed_json = json.loads(sample_text)
        except json.JSONDecodeError:
            # 若解析失敗，嘗試從 Markdown 內提取 JSON
            pattern = r"```json(.*?)```"
            json_part = re.search(pattern, sample_text, re.DOTALL)

            extracted_json = json_part.group(1).strip() if json_part else None
            if not extracted_json:
                self.get_logger().error("無法提取 JSON！")
                return False

            try:
                parsed_json = json.loads(extracted_json)
            except json.JSONDecodeError:
                self.get_logger().error("解析 JSON 失敗！")
                return False

        # 成功解析後寫入檔案
        with open(output_file_path, 'w', encoding='utf-8') as json_file:
            json.dump(parsed_json, json_file, indent=4)

        self.get_logger().info(f"JSON 已儲存至 {output_file_path}")
        return True  # 回傳 True 表示存儲成功


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    # rclpy.spin(node)
    if rclpy.ok():  # 確保 ROS 2 還在運行才 spin
        rclpy.spin(node)

    # print("debug message 3")
    # node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
