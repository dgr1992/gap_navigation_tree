import networkx as nx
import matplotlib.pyplot as plt

class GraphVisualisation:
    def __init__(self):
        self.G = nx.Graph()
        self.G.add_node('r')
        self.redraw = False
        
    def add_node_to_root(self, node):
        self.G.add_node(node.id)
        self.G.add_edge('r', node.id)

    def add_node(self, parent_node, node):
        self.G.add_node(node.id)
        self.G.add_edge(parent_node.id, node.id)

    def remove_node(self, node):
        self.G.remove_node(node.id)

    def remove_edge_between_root_and_node(self, node):
        self.G.remove_edge('r', node)

    def remove_edge_between_nodes(self, node_1, node_2):
        self.G.remove_edge(node_1.id, node_2.id)

    def add_edge_between_root_and_node(self, node):
        self.G.add_edge('r', node)

    def add_edge_between_nodes(self, node_1, node_2):
        self.G.add_edge(node_1.id, node_2.id)

    def draw(self):
        plt.clf()
        nx.draw(self.G, with_labels=True, font_weight='bold')
        plt.draw()
        self.redraw = False

    def show(self):
        plt.figure("Graph")
        plt.ion()
        plt.subplot(111)
        plt.show()

if __name__ == "__main__":
    g = GraphVisualisation()
    g.show()