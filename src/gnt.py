import rospy

from critical_event_enum import CriticalEventEnum
from tree_node import TreeNode
from gap_sensor.msg import MovedGaps, GapMove

class GapNavigationTree:
    
    def __init__(self):
        rospy.init_node('gap_navigation_tree')

        self.root = [None] * 360

        self.visualise = True

        self._init_publishers()
        self._init_subscribers()

    def _init_subscribers(self):
        """
        Initialise subscribers
        """
        self.sub_critical_event = rospy.Publisher("critical_event", CriticalEvent, self._receive_critical_event queue_size=5)
        self.sub_gap_move = rospy.Publisher("gap_move", GapMove, self._handle_move_event, queue_size=5)

    def _init_publishers(self):
        """
        Initialise publishersubscribers
        """
        pass

    def run(self):
        """
        Main loop
        """
        if self.visualise:
            self.graph_visualisation = GraphVisualisation()
            self.graph_visualisation.show()

        while not rospy.is_shutdown():

            if self.visualise and self.graph_visualisation.redraw:
                self.graph_visualisation.draw()

    def _receive_critical_event(self, data):
        """
        Receive critical event
        """
        event = CriticalEventEnum(data.event_type)

        if event == CriticalEventEnum.APPEAR:
            self._handle_appear(data.angle_new)
        
        if event == CriticalEventEnum.DISAPPEAR:
            self._handle_disappear(data.angle_new)

        if event == CriticalEventEnum.SPLIT:
            self._handle_split(data.angle_old, data.angle_new)

        if event == CriticalEventEnum.MERGE:
            self._handle_merge(data.angle_old, data.angle_new)
      
    def _handle_move_event(self,data):
        """
        Handle the move event. Update the position of the gap.

        data (GapMove): angle where the gap appeared before and the angle where the gap is now
        """
        angle_old = data.angle_old
        angle_new = data.angle_new

        self.root[angle_new] = self.root[angle_old]
        self.root[angle_old] = None

    def _handle_appear(self, angle):
        """
        Gap appear at angle. Create a new node representing the gap.

        Parameters:
        angle (int): Angle at which the gap appeared
        """
        node = TreeNode()
        self.root[angle] = node

        # update graph
        self.graph_visualisation.add_node_to_root(node)
        self.graph_visualisation.redraw = True

    def _handle_disappear(self, angle):
        """
        Gap at angle disappear.
        """
        if len(self.root[index].children) > 0:
            self._recursive_remove_childs_from_graph(self.root[index])

        # update graph
        self.graph_visualisation.remove_node(self.root[index])
        self.graph_visualisation.redraw = True

        self.root[angle] = None

    def _recursive_remove_childs_from_graph(self, node):
        """
        Check the node for childs and remove them from the graph
        """
        if len(node.children) != 0:
            break
        
        # remove all connected nodes
        for child in nodes.children:
            self._recursive_remove_childs(child)
            self.graph_visualisation.remove_node(child)

    def _handle_split(self, angle_1, angle_2):
        """
        Gap at angle_splits into angle_1 and angle_2
        """        
        angle = None

        # order angles asc
        start = angle_1
        end = angle_2
        if angle_2 < start:
            start = angle_2
            end = angle_1

        # find the gap that was splitted
        for i in range(start, (end + 1) % len(self.root)):
            if self.root[i] != None:
                angle = i

        # the node representing the splitted gap has childs if the appeard gaps are kown
        if len(self.root[index].children) == 2:
            self._gap_split_existing(angle, angle_1, angle_2)
        else:
            self._gap_split_new(angle, angle_1, angle_2)
            
    def _gap_split_existing(self, angle_splitted, angle_1, angle_2):
        """
        From node at angle_splitted get the childs and place them at angle_1 and angle_1.

        Parameters:
        angle_splitted (int): Angle of the gap that splitted
        angle_1 (int): First angle where the first child needs to be placed in the tree
        angle_2 (int): Second angle where the second child needs to be placed in the tree
        """
        # get the splitting node
        node_splitting = self.root[index]
        
        # remove node from tree 
        self.root[angle_splitted] = None

        # remove edge bewteen root and the splitting node
        self.graph_visualisation.remove_edge_between_root_and_node(node_splitting)

        # updated the tree by adding the nodes resulted from split
        if angle_2 < angle_1:
            self.root[angle_2] = node_splitting.nodes[0]
            self.root[angle_1] = node_splitting.nodes[1]
        else:
            self.root[angle_1] = node_splitting.nodes[0]
            self.root[angle_2] = node_splitting.nodes[1]

        # connect the nodes to root
        self.graph_visualisation.add_edge_between_root_and_node(self.root[angle_1])
        self.graph_visualisation.add_edge_between_root_and_node(self.root[angle_2])

        self.graph_visualisation.redraw = True
    
    def _gap_split_new(self, angle_splitted, angle_1, angle_2):
        """
        Create new nodes for angle_1 and angle_2.

        Parameters:
        angle_splitted (int): Angle of the gap that splitted
        angle_1 (int): First angle for new node
        angle_2 (int): Second angle for new node
        """

        # remove node, which represents the splitted gap, from the graph
        self.graph_visualisation.remove_node(self.root[angle_splitted])
        self.root[angle_splitted] = None

        # create new nodes and add to the root
        node1 = TreeNode()
        self.root[angle_1] = node1

        node2 = TreeNode()
        self.root[angle_2] = node2

        # update graph
        self.graph_visualisation.add_node_to_root(self.root[angle_1])
        self.graph_visualisation.add_node_to_root(self.root[angle_2])
        self.graph_visualisation.redraw = True

    def _handle_merge(self, angle_1, angle_2):
        """
        Gap at angle_1 and angle_2 merge into angle_2.

        angle_1 (int):
        angle_2 (int):
        """
        # get the merging nodes
        node_1 = self.root[angle_1]
        node_2 = self.root[angle_2]
        
        # merge nodes
        node = TreeNode()
        if angle_2 < angle_1:
            node.nodes.append(node_2)
            node.nodes.append(node_1)
        else:
            node.nodes.append(node_1)
            node.nodes.append(node_2)
        
        # delete old from depth jumps
        self.root[angle_1] = None
        # add the merged node
        self.root[angle_2] = node


if __name__ == "__main__":
    try:
        gnt = GapNavigationTree()
        gnt.run()
    except rospy.ROSInterruptException as ex:
        print(str(ex))