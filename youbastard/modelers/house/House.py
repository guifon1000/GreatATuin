import json







class House(dict):
    def __init__(self, **kwargs):
        super(House, self).__init__(**kwargs)
    def write_json(self, name):
        json.dump(self, open(name+'.json', 'w'))

