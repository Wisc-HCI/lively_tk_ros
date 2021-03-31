from lively_ik.configuration.config_manager import ConfigManager
from lively_ik_core import parse_config_data, LivelyIK
import yaml
import time
import math
from datetime import datetime
from pprint import PrettyPrinter
import os
import json

pprinter = PrettyPrinter()
pprint = lambda content: pprinter.pprint(content)

script_dir = os.path.dirname(__file__)
config_file = os.path.join(script_dir,'../config/nao_nice.json')

with open(config_file) as handle:
    data = yaml.safe_load(handle)

cm = ConfigManager(lambda: print(f'\033[93mProgress: {cm.meta["nn_progress"]}\033[0m'))
print('Updating to config')
cm.update(data)
pprint(cm.meta)
print('Beginning Training')
cm.train_nn()

cm.update({field:data[field] for field in ['objectives','goals','modes']})

config_file = os.path.join(script_dir,'../config/nao_nice_nn2.json')
with open(config_file, 'w') as handle:
    json.dump(cm.data,handle)

exit()

# elements = []
#
# for field in cm._fields.keys():
#     elements.append({'data':{'id': field, 'label': field}})
#     for derivation in cm._fields[field]['dependencies']:
#         elements.append({'data':{'source': field, 'target': derivation},'classes':'dep'})
#     # for guard in cm._fields[field]['guards']:
#     #     elements.append({'data':{'source': field, 'target': guard},'classes':'guard'})
#
# import dash
# import dash_core_components as dcc
# import dash_html_components as html
# import dash_cytoscape as cyto
# from dash.dependencies import Input, Output
# import plotly.express as px
#
# app = dash.Dash(__name__)
#
# stylesheet = [
#     {
#         "selector": 'node', #For all nodes
#         'style': {
#             "opacity": 0.9,
#             "label": "data(label)", #Label of node to display
#             "background-color": "#07ABA0", #node color
#             "color": "#008B80" #node label color
#         }
#     },
#     {
#         "selector": 'edge.dep', #For all edges
#         "style": {
#             "target-arrow-color": "#C5D3E2", #Arrow color
#             "target-arrow-shape": "triangle", #Arrow shape
#             "line-color": "#C5D3E2", #edge color
#             'arrow-scale': 2, #Arrow size
#             'curve-style': 'bezier' #Default curve-If it is style, the arrow will not be displayed, so specify it
#         }
#     },
#     {
#         "selector": 'edge.guard', #For all guard edges
#         "style": {
#             "target-arrow-color": "##F8C0CF", #Arrow color
#             "target-arrow-shape": "triangle", #Arrow shape
#             "line-color": "##F8C0CF", #edge color
#             'arrow-scale': 2, #Arrow size
#             'curve-style': 'bezier' #Default curve-If it is style, the arrow will not be displayed, so specify it
#         }
#     }
# ]
#
# app.layout = html.Div([
#     html.P("Dash Cytoscape:"),
#     cyto.Cytoscape(
#         id='cytoscape',
#         elements=elements,
#         layout={'name': 'cose'},#{'name': 'breadthfirst', 'roots':'#urdf, #mode_control, #mode_environment, #robot_link_radius, #static_environment, #base_link_motion_bounds'},
#         style={'width': '1280px', 'height': '800px'},
#         stylesheet=stylesheet
#     )
# ])
#
# app.run_server(debug=True)
# exit()
#
# cm.load({'urdf':data['urdf']})
# pprint(cm.simplified())
# pprint(cm.meta)

# cm.load(data)
# pprint(cm.simplified())
# pprint(cm.meta)
# exit()
config = parse_config_data(data)
solver = LivelyIK(config)
print(cm.current['markers'])
print(solver.solve(config.default_goals,9.0))
exit()



dfdata = {joint:[] for joint in data['joint_ordering']}

num_solves = 1000

solver_goals = config.default_goals

print('running with liveliness')
# for i in range(num_solves):
while True:
    try:
        time = datetime.utcnow().timestamp()
        # print(time)
        trans,sol = solver.solve(solver_goals,time)
        print(sol)
        for j,v in enumerate(sol):
            dfdata[data['joint_ordering'][j]].append(sol[j])
    except (KeyboardInterrupt, SystemExit):
        break

df = pd.DataFrame(dfdata,index=list(range(len(dfdata['panda_joint1']))))
df.plot.line()
plt.show()
