from lively_ik.configuration.config_manager import ConfigManager
from lively_ik_core import parse_config_data, LivelyIK
from pprint import PrettyPrinter
import yaml
from datetime import datetime
import rclpy
import math
import os

pprinter = PrettyPrinter()
pprint = lambda content: pprinter.pprint(content)

script_dir = os.path.dirname(__file__)
config_file = os.path.join(script_dir,'../config/nao_test.json')

with open(config_file) as handle:
    data = yaml.safe_load(handle)

parsed = parse_config_data(data)
# print(parsed)

lik = LivelyIK(parsed)

while True:
    try:
        time = datetime.now().timestamp()
        # print(time)
        trans,sol = lik.solve(parsed.default_inputs,time,max_retries=0,max_iterations=500)
        print(f'\033[93m{trans}\033[0m {sol}')
    except (KeyboardInterrupt, SystemExit):
        break
