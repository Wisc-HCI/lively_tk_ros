from lively_ik.configuration.config_manager import ConfigManager
from lively_ik_core import parse_config_data, LivelyIK
import yaml
import time
import math
from datetime import datetime
from pprint import PrettyPrinter
import os

pprinter = PrettyPrinter()
pprint = lambda content: pprinter.pprint(content)

script_dir = os.path.dirname(__file__)
config_file = os.path.join(script_dir,'../config/panda.json')

with open(config_file) as handle:
    data = yaml.safe_load(handle)

cm = ConfigManager()
pprint(cm.test_derive('starting_config'))
cm.update(data)
pprint(cm.test_derive('starting_config'))
