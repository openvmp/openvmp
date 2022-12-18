class Config:
    obj = {}

    def __init__(self, obj):
        self.obj = obj

    def get_joints(self):
        if not "joints" in self.obj:
            return []
        return self.obj["joints"]

    def get_buses(self):
        if not "buses" in self.obj:
            return []
        return self.obj["buses"]

    def get_cameras(self):
        if not "cameras" in self.obj:
            return []
        return self.obj["cameras"]
