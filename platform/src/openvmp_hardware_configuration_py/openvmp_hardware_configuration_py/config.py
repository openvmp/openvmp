class Config:
    obj = {}

    def __init__(self, obj):
        self.obj = obj

    def get_joints(self):
        return self.obj["joints"]

    def get_buses(self):
        return self.obj["buses"]
