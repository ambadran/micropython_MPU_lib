#TODO: must make an abstract class for this
class Register(device):
    def __init__(self, address):
        self.address = address
        self.value: bin
        
    @property
    def value(self):
        pass # use the register_write function in device parent class
        