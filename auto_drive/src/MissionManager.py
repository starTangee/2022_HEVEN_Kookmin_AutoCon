import warnings

from Database import Database
from Mission import Mission

class MissionManager:
    def __init__(self, db):
        self.missions = dict()
        self.db = db
        self.mission_keys = list()
        self.mission_idx = None
        self.current_mission_key = None

    def add_mission(self, key, mission=Mission):
        if key not in self.mission_keys:
            warnings.warn("The new key %s is not registered.\
                 Therefore, NO mission will be added." % key)
        else:
            mission.key = key
            self.missions[key] = mission
    
    def main(self):
        # Get mission key
        current_mission = self.missions[self.current_mission_key]
        # Execute main function of current mission
        car_angle, car_speed = current_mission.main()
        if current_mission.mission_end(): 
            self.next_mission()
        return car_angle, car_speed

    def next_mission(self):
        if self.mission_idx >= len(self.mission_keys):
            print("All missions are complete. System will stop.")
        else:
            self.mission_idx += 1
            self.current_mission_key = self.mission_keys[self.mission_idx]