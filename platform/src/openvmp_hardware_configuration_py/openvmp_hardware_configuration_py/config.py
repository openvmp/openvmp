class Config:
    obj = {}

    def __init__(self, obj):
        self.obj = obj

    def get_modbus_rtu(self):
        return self.obj["interfaces"]["modbus_rtu"]
