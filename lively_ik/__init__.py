import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

BASE = get_package_share_directory('lively_ik')#str(Path(os.path.realpath(__file__)).parent.parent)
