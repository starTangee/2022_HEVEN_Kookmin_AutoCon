from Database import Database

class Mission():
    def __init__(self, db):
        self.db = db
        self.key = None
    
    def main(self):
        pass
    
    def mission_end(self):
        pass

    def __str__(self):
        if self.key is None:
            return "None"
        else:
            return self.key + " Mission"


if __name__ == "__main__":
    mission = Mission()
    Mission.main()