import dash
from collections import deque
import plotly.graph_objs as go
import random
import plotly
import dash_html_components as html
import dash_core_components as dcc
from dash.dependencies import Output, Input
from time import sleep

"""
1. rqt_pytrees are for ros 1
There's a ros2 implementation of py_trees viewer as well. It uses pyqt & a light weight JS library as the backend 
However, all message passing in ros 2 needs to be replaced by ros1.
"""

def test_graphviz():
    """
    Live visualization is crappy
    """
    from graphviz import Digraph

    dot = Digraph(comment='The Round Table', format='png', strict=True)

    dot.node('A', 'one', color="red")
    dot.node('B', 'two', color="red")
    dot.node('C', 'three', color="red")
    dot.node('D', 'four', color="red")
    dot.node('E', 'five', color="red")
    dot.node('F', 'six', color="red")
    dot.node('G', 'device1', color="sienna")
    dot.node('H', 'device2', color="green")

    dot.edges(['AB', 'AG', 'AD', 'AE', 'BH', 'BC', 'BF'])
    dot.edges(['BA', 'GA', 'DA', 'EA', 'HB', 'CB', 'FB'])
    dot.edges(['CD', 'DC', 'EF', 'FE'])

    while (True):
        dot.edge('H', 'B', color="black")
        sleep(.5)
        dot.render("TestGraphLiveUpdate")
        dot.view()
        dot.edge('H', 'B', color="green")
        sleep(.5)
        dot.render("TestGraphLiveUpdate")
        dot.view()


def test_pyvis():
    """
    Showing stuff as static HTML, not good for live visualization
    """
    from pyvis.network import Network
    import networkx as nx
    net = Network()
    net.add_nodes([1, 2, 3], value=[10, 100, 400],
                  title=['I am node 1', 'node 2 here', 'and im node 3'],
                  x=[21.4, 54.2, 11.2],
                  y=[100.2, 23.54, 32.1],
                  label=['NODE 1', 'NODE 2', 'NODE 3'],
                  color=['#00ff1e', '#162347', '#dd4b39'])
    net.show()


def test_plotly():
    """
    Plotly will open up a new page in browser, but does not support live updates.
    """
    import plotly.express as px
    # using the iris dataset
    df = px.data.iris()
    # plotting the line chart
    fig = px.line(df, y="sepal_width",)
    # showing the plot
    fig.show()

    sleep(4)

    import igraph
    from igraph import Graph, EdgeSeq
    nr_vertices = 25
    v_label = list(map(str, range(nr_vertices)))
    G = Graph.Tree(nr_vertices, 2)  # 2 stands for children number
    lay = G.layout('rt')

    position = {k: lay[k] for k in range(nr_vertices)}
    Y = [lay[k][1] for k in range(nr_vertices)]
    M = max(Y)

    es = EdgeSeq(G)  # sequence of edges
    E = [e.tuple for e in G.es]  # list of edges

    L = len(position)
    Xn = [position[k][0] for k in range(L)]
    Yn = [2 * M - position[k][1] for k in range(L)]
    Xe = []
    Ye = []
    for edge in E:
        Xe += [position[edge[0]][0], position[edge[1]][0], None]
        Ye += [2 * M - position[edge[0]][1],
               2 * M - position[edge[1]][1], None]

    labels = v_label

    import plotly.graph_objects as go
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=Xe,
                             y=Ye,
                             mode='lines',
                             line=dict(color='rgb(210,210,210)', width=1),
                             hoverinfo='none'
                             ))
    fig.add_trace(go.Scatter(x=Xn,
                             y=Yn,
                             mode='markers',
                             name='bla',
                             marker=dict(symbol='circle-dot',
                                         size=18,
                                         color='#6175c1',  # '#DB4551',
                                         line=dict(
                                             color='rgb(50,50,50)', width=1)
                                         ),
                             text=labels,
                             hoverinfo='text',
                             opacity=0.8
                             ))
    fig.show()
    print("hellow")


def test_plotly_and_dash():
    """
    Yes! This updates a graph live
    """
    import dash
    from dash.dependencies import Output, Input
    import dash_core_components as dcc
    import dash_html_components as html
    import plotly
    import random
    import plotly.graph_objs as go
    from collections import deque

    X = deque(maxlen = 20)
    X.append(1)

    Y = deque(maxlen = 20)
    Y.append(1)

    app = dash.Dash(__name__)

    app.layout = html.Div(
        [
            dcc.Graph(id = 'live-graph', animate = True),
            dcc.Interval(
                id = 'graph-update',
                interval = 1000,
                n_intervals = 0
            ),
        ]
    )

    count = 0
    @app.callback(
        Output('live-graph', 'figure'),
        [ Input('graph-update', 'n_intervals') ]
    )

    def update_graph_scatter(n):
        nonlocal count

        # X.append(X[-1]+1)
        # Y.append(Y[-1]+Y[-1] * random.uniform(-0.1,0.1))
        #
        # data = plotly.graph_objs.Scatter(
        #         x=list(X),
        #         y=list(Y),
        #         name='Scatter',
        #         mode= 'lines+markers'
        # )
        #

        import igraph
        from igraph import Graph, EdgeSeq
        nr_vertices = 25
        v_label = list(map(str, range(nr_vertices)))
        G = Graph.Tree(nr_vertices, 2)  # 2 stands for children number
        lay = G.layout('rt')

        position = {k: lay[k] for k in range(nr_vertices)}
        Y = [lay[k][1] for k in range(nr_vertices)]
        M = max(Y)

        es = EdgeSeq(G)  # sequence of edges
        E = [e.tuple for e in G.es]  # list of edges

        L = len(position)
        Xn = [position[k][0] for k in range(L)]
        Yn = [2 * M - position[k][1] for k in range(L)]
        Xe = []
        Ye = []
        for edge in E:
            Xe += [position[edge[0]][0], position[edge[1]][0], None]
            Ye += [2 * M - position[edge[0]][1],
                   2 * M - position[edge[1]][1], None]

        labels = v_label

        import plotly.graph_objects as go
        edges = go.Scatter(x=Xe,
                                 y=Ye,
                                 mode='lines',
                                 line=dict(color='rgb(210,210,210)', width=1),
                                 hoverinfo='none'
                                 )
        nodes = go.Scatter(x=Xn,
                                 y=Yn,
                                 mode='markers',
                                 name='bla',
                                 marker=dict(symbol='circle-dot',
                                             size=18,
                                             color='#6175c1',  # '#DB4551',
                                             line=dict(
                                                 color='rgb(50,50,50)', width=1)
                                             ),
                                 text=labels,
                                 hoverinfo='text',
                                 opacity=0.8
                                 )

        count += 0
        return {'data': [edges, nodes],
                'layout' : go.Layout(xaxis=dict(range=[0, 10]),yaxis = dict(range = [0,10]),)}

    app.run_server()

if __name__ == "__main__":
    # test_pyvis()
    # test_plotly()
    test_plotly_and_dash()
