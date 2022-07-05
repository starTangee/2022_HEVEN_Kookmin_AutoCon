import warnings

from Database import Database
from Mission import Mission

class MissionManager:
    def __init__(self, db: Database):
        self.missions = dict()
        self.db = db
        self.mission_keys = list()
        self.mission_idx = None
        self.current_mission_key = None

    def add_mission(self, key, mission: Mission):
        if key not in self.mission_keys:
            warnings.warn("The new key %s is not registered.\
                 Therefore, NO mission will be added." % key)
        else:
            mission.key = key
            self.missions[key] = mission
    
    def main(self):
        current_mission = self.missions[self.current_mission_key] # 미션 번호를 수신함
        current_mission.main() # 미션 탈출조건이 만족될 때까지 해당 미션 수행
        if current_mission.mission_end(): 
            # 정상적으로 미션이 끝났을 때만 다음 미션으로 넘어감.
            self.next_mission()
    
    def next_mission(self):
        if self.mission_idx >= len(self.mission_keys):
            print("All missions are complete. System will stop.")
            self.db.flag.system_stop = True
        else:
            self.mission_idx += 1
            self.current_mission_key = self.mission_keys[self.mission_idx]