from lively_ik.configuration.config_manager import ConfigManager
from lively_ik_core import parse_config_data, LivelyIK
import yaml
import time
import math
from datetime import datetime
from pprint import PrettyPrinter
import os

import dash
import dash_core_components as dcc
import dash_html_components as html
import dash_cytoscape as cyto
from dash.dependencies import Input, Output
import plotly.express as px

cyto.load_extra_layouts()

pprinter = PrettyPrinter()
pprint = lambda content: pprinter.pprint(content)

script_dir = os.path.dirname(__file__)
config_file = os.path.join(script_dir,'../config/panda.json')

with open(config_file) as handle:
    data = yaml.safe_load(handle)

# parsed = parse_config_data(data)
# print(parsed)
#
# lik = LivelyIK(parsed)
# print(lik.solve(parsed.default_inputs,9.0,max_retries=5))

# print(data['objectives'])
cm = ConfigManager()
# pprint(cm._update_pairs)

elements = []

for field in cm._fields.keys():
    elements.append({'data':{'id': field, 'label': field}})
    for dep in cm._fields[field]['dependencies']:
        elements.append({'data':{'source': dep, 'target': field},'classes':'dep'})
    # for guard in cm._fields[field]['guards']:
    #     elements.append({'data':{'source': field, 'target': guard},'classes':'guard'})


app = dash.Dash(__name__)

stylesheet = [
    {
        "selector": 'node', #For all nodes
        'style': {
            "opacity": 0.9,
            "label": "data(label)", #Label of node to display
            "background-color": "#07ABA0", #node color
            "color": "#008B80" #node label color
        }
    },
    {
        "selector": 'edge.dep', #For all edges
        "style": {
            "target-arrow-color": "#C5D3E2", #Arrow color
            "target-arrow-shape": "triangle", #Arrow shape
            "line-color": "#C5D3E2", #edge color
            'arrow-scale': 2, #Arrow size
            'curve-style': 'bezier' #Default curve-If it is style, the arrow will not be displayed, so specify it
        }
    },
    {
        "selector": 'edge.guard', #For all guard edges
        "style": {
            "target-arrow-color": "##F8C0CF", #Arrow color
            "target-arrow-shape": "triangle", #Arrow shape
            "line-color": "##F8C0CF", #edge color
            'arrow-scale': 2, #Arrow size
            'curve-style': 'bezier' #Default curve-If it is style, the arrow will not be displayed, so specify it
        }
    }
]

app.layout = html.Div([
    html.P("Dash Cytoscape:"),
    cyto.Cytoscape(
        id='cytoscape',
        elements=elements,
        layout={'name': 'dagre'},#{'name': 'breadthfirst', 'roots':'#urdf, #mode_control, #mode_environment, #robot_link_radius, #static_environment, #base_link_motion_bounds'},
        style={'width': '1280px', 'height': '800px'},
        stylesheet=stylesheet
    )
])

app.run_server(debug=True)
