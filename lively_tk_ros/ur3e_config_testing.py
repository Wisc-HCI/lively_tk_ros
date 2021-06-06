from lively_tk_ros.configuration.config_manager import ConfigManager
import os
from pprint import PrettyPrinter

pprinter = PrettyPrinter()
pprint = lambda content: pprinter.pprint(content)

script_dir = os.path.dirname(__file__)
urdf_file = os.path.join(script_dir,'./launch/ur3e.xml')

with open(urdf_file) as file:
    urdf = file.read().replace('\n', '')

print(urdf)

cm = ConfigManager()
print('created manager')
cm.update({'urdf':urdf})
print('updated urdf')

