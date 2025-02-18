# network_control/supabase_manager.py
# 2025 CGV Capstone
# Author: 22100600 이현서 <hslee@handong.ac.kr>

import os 
from supabase import create_client, Client
from supabase.client import ClientOptions

class SupabaseManager:
    robot_id: int = None
    url: str = None
    key: str = None
    supabase: Client = None

    def __init__(self):
        self.robot_id = os.environ.get("ROBOT_ID")     # 환경변수 ROBOT_ID를 읽어옴
        self.url      = os.environ.get("SUPABASE_URL") # 환경변수 SUPABASE_URL을 읽어옴
        self.key      = os.environ.get("SUPABASE_KEY") # 환경변수 SUPABASE_KEY를 읽어옴
        self.supabase = create_client(self.url, self.key)
    
    def update_robot_status(self, data: dict):
        """
        로봇의 상태를 Supabase DB에 업로드한다.
        :param table_name: "robots" (fixed)
        :param data: {"id": robot_id, "status": 0, "battery": 100, "position": [pos_x, pos_y]}
        """

        try:
            # Scale transform to match the 2D map dimensions
            if "position" in data:
                data["position"] = [data["position"][0] * 10.0, data["position"][1] * 10.0]

            query = (self.supabase.table("robots")
                     .update(data)
                     .eq("id", self.robot_id)
                     .execute())
        
        except Exception as e:
            print(f"[SupabaseManager] Robot status update error: {e}")

    
    def fetch_request(self):
        """
        DB에서 새로운 명령(목표지점, 비상정지 등)을 받아온다.
        :param table_name: "request" (fixed)
        :param data: {"id": Sequential Number (for Primary Key), "robot_id": robot_id, "status": 0, "battery": 100, "position": [pos_x, pos_y]}
        :return: 조회 결과
        """

        print("Fetch request... of ROBOT_ID="+str(self.robot_id))

        try:
            query = (self.supabase.table("request")
                     .select("*")
                     .eq("robot_id", self.robot_id)
                     .order("request_time", desc=True)
                     .limit(1)
                     .execute())
            converted = query.data[0]
            if "goal_position" in converted:
                # Scale transform to match the SLAM map dimensions
                converted["goal_position"] = [converted["goal_position"][0] / 10.0, converted["goal_position"][1] / 10.0]
                print(converted)

                return converted

        except Exception as e:
            print(f"[SupabaseManager] Request fetch error: {e}")
            return None
        
    def create_response(self, data: dict):
        """
        받은 명령에 대한 응답을 DB에 업로드한다.
        :param table_name: response (fixed)
        :param data: {"id": Sequential Number (for Primary Key), "robot_id": robot_id, "complete": FALSE, "geterated_path": [[pos_x1, pos_y1], [pos_x2, pos_y2], ...]}
        :return: 조회 결과 목록
        """

        try:
            data["robot_id"] = self.robot_id
            if "generated_path" in data:
                # Scale transform to match the 2D map dimensions
                data["generated_path"] = [[element * 10.0 for element in row] for row in data["generated_path"]]
            query = (self.supabase.table("response")
                     .insert(data)
                     .execute())
        
        except Exception as e:
            print(f"[SupabaseManager] Response creation error: {e}")
            return None
        