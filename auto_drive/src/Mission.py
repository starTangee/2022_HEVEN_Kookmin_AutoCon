from Database import Database
from abc import abstractclassmethod, ABCMeta

class Mission(metaclass=ABCMeta):
    def __init__(self, db: Database):
        self.db = db
        self.key = None
    
    @abstractclassmethod
    def main(self): # 미션 수행 함수
        pass
    
    @abstractclassmethod
    def mission_end(self): # 탈출조건 검사 함수
        pass

    def __str__(self):
        if self.key is None:
            return "None"
        else:
            return self.key + " Mission"


if __name__ == "__main__":
    mission = Mission()
    Mission.main()