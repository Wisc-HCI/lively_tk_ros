

class App(object):
    def __init__(self,node,name,code):
        self.node = node
        self.name = name
        self.code = code


    @property
    def info(self):
        return {'name':self.name,'code':self.code}

    def update(self, data):
        return {'success':True}

    def log(self,message):
        self.node.get_logger().info(message)

    def warn(self,message):
        self.node.get_logger().warn(message)

    def error(self,message):
        self.node.get_logger().error(message)
