import os
from pathlib import Path

#BASE = os.path.realpath(os.path.relpath('../config/', os.path.dirname(__file__)))
#BASE = os.path.abspath('../config/')
#BASE = os.path.dirname(__file__)
BASE = str(Path(os.path.realpath(__file__)).parent.parent)
