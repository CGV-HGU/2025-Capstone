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
    res_scale_factor = None

    def __init__(self):
        self.robot_id = os.environ.get("ROBOT_ID")     # 환경변수 ROBOT_ID를 읽어옴
        self.url      = os.environ.get("SUPABASE_URL") # 환경변수 SUPABASE_URL을 읽어옴
        self.key      = os.environ.get("SUPABASE_KEY") # 환경변수 SUPABASE_KEY를 읽어옴
        self.supabase = create_client(self.url, self.key)
        self.res_scale_factor = 41.5 #20.0
    
    def update_robot_status(self, data: dict):
        """
        로봇의 상태를 Supabase DB에 업로드한다.
        :param table_name: "robots" (fixed)
        :param data: {"id": robot_id, "status": 0, "battery": 100, "position": [pos_x, pos_y]}
        """

        try:
            # Scale transform to match the 2D map dimensions
            if "position" in data:
                data["position"] = [data["position"][0] * self.res_scale_factor, data["position"][1] * self.res_scale_factor]

            data["battery"] = 100

            query = (self.supabase.table("robots")
                     .update(data)
                     .eq("id", self.robot_id)
                     .execute())
        
        except Exception as e:
            print(f"[SupabaseManager] Robot status update error: {e}")

    
    def fetch_request(self):
        """
        DB에서 새로운 명령(목표지점, 비상정지 등)을 받아온다.
        :return: dict or None
        """
        try:
            result = (
                self.supabase
                    .table("request")
                    .select("*")
                    .eq("robot_id", self.robot_id)
                    .order("request_time", desc=True)
                    .limit(1)
                    .execute()
            )
            if not result.data:
                return None

            req = result.data[0]

            # Scale only if goal_position is actually an array
            gp = req.get("goal_position")
            if gp is not None:
                req["goal_position"] = [gp[0] / self.res_scale_factor,
                                        gp[1] / self.res_scale_factor]

            return req

        except Exception as e:
            print(f"[SupabaseManager] Request fetch error: {e}")
            return None

        
    def create_response(self, data: dict):
        """
        받은 명령에 대한 응답을 DB에 업로드한다.
        :param table_name: response (fixed)
        :param data: {"id": Sequential Number (for Primary Key), "robot_id": robot_id, "complete": FALSE}
        :return: 조회 결과 목록
        """

        try:
            data["robot_id"] = self.robot_id
            query = (self.supabase.table("response")
                     .insert(data)
                     .execute())
        
        except Exception as e:
            print(f"[SupabaseManager] Response creation error: {e}")
            return None
        