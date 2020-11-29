import networkx as nx
import matplotlib.pyplot as plt
from tree_node import TreeNode

class GraphVisualisation:
    def __init__(self):
        self.G = nx.Graph()
        self.G.add_node('r')
        self.redraw = False
        self.fig = None
        
    def add_node_to_root(self, node):
        if node == None:
            return
        self.G.add_node(node.id)
        self.G.add_edge('r', node.id)

    def add_node(self, parent_node, node):
        if parent_node == None or node == None:
            return
        self.G.add_node(node.id)
        self.G.add_edge(parent_node.id, node.id)

    def remove_node(self, node):
        if node == None:
            return
        self.G.remove_node(node.id)

    def remove_edge_between_root_and_node(self, node):
        if node == None:
            return
        self.G.remove_edge('r', node.id)

    def remove_edge_between_nodes(self, node_1, node_2):
        if node_1 == None or node_2 == None:
            return
        self.G.remove_edge(node_1.id, node_2.id)

    def add_edge_between_root_and_node(self, node):
        if node == None:
            return
        self.G.add_edge('r', node.id)

    def add_edge_between_nodes(self, node_1, node_2):
        if node_1 == None or node_2 == None:
            return
        self.G.add_edge(node_1.id, node_2.id)

    def draw(self):
        plt.clf()
        
        plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
        plt.tick_params(axis='y', which='both', right=False, left=False, labelleft=False)
        for pos in ['right','top','bottom','left']:
            plt.gca().spines[pos].set_visible(False)
        
        nx.draw_networkx(self.G, with_labels=True, font_weight='bold')
        self.fig.canvas.draw()
        self.redraw = False

    def show(self):
        #plt.figure("Graph")
        #plt.ion()
        #plt.subplot(111)
        #plt.show()
        self.fig = plt.figure("Graph", figsize=(8,6))
        plt.ion()
        plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
        plt.tick_params(axis='y', which='both', right=False, left=False, labelleft=False)
        for pos in ['right','top','bottom','left']:
            plt.gca().spines[pos].set_visible(False)
        #nx.draw_networkx(self.G, with_labels=True, font_weight='bold')
        plt.show()

if __name__ == "__main__":
    g = GraphVisualisation()
    g.show()
    n0 = TreeNode()
    n1 = TreeNode()
    n2 = TreeNode()
    n3 = TreeNode()
    n4 = TreeNode()
    g.add_node_to_root(n0)
    g.draw()
    g.add_node_to_root(n1)
    g.add_node_to_root(n2)
    g.draw()
    g.remove_edge_between_root_and_node(n1)
    g.remove_edge_between_root_and_node(n2)
    g.add_node_to_root(n3)
    g.add_edge_between_nodes(n3,n1)
    g.add_edge_between_nodes(n3,n2)
    g.draw()
    g.add_node_to_root(n4)
    g.draw()
    raw_input("Press Enter to continue...")