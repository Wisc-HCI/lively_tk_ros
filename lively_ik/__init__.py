import os
from pathlib import Path

BASE = str(Path(os.path.realpath(__file__)).parent.parent)

def get_base():
    return BASE
