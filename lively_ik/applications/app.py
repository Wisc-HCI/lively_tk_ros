

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

    def process(self, data):
        for i in range(10):
            yield {'success':True, 'value':i}

    def log(self,message):
        self.node.get_logger().info(message)

    def warn(self,message):
        self.node.get_logger().warn(message)

    def error(self,message):
        self.node.get_logger().error(message)

    def handle_goal_update(self,request,response):
        return response
