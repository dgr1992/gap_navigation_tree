import rospy
import threading
import copy
from critical_event_enum import CriticalEventEnum
from tree_node import TreeNode
from gap_sensor.msg import MovedGaps, GapMove, CriticalEvent, CriticalEvents, CollectionCriticalAndMoved
from gap_navigation_tree.msg import GapTreeNode, GapTreeNodes
from graph_visualisation import GraphVisualisation
from gap_visualiser import GapVisualiser

class GapNavigationTree:
    
    def __init__(self):
        rospy.init_node('gap_navigation_tree')

        self.root = [None] * 360

        self.visualise = True

        self._init_publishers()
        self._init_subscribers()

        self.lock = threading.Lock()

    def _init_subscribers(self):
        """
        Initialise subscribers
        """
        self.sub_collection = rospy.Subscriber("collection_critical_and_moved", CollectionCriticalAndMoved, self._handle_collection, queue_size=15)

    def _init_publishers(self):
        """
        Initialise publishersubscribers
        """
        self.pub_tree = rospy.Publisher("gap_tree", GapTreeNodes, queue_size=5)

    def run(self):
        """
        Main loop
        """
        if self.visualise:
            self.graph_visualisation = GraphVisualisation()
            self.graph_visualisation.show()
            self.gap_visualisation = GapVisualiser("GNT -360")

        while not rospy.is_shutdown():

            if self.visualise:
                if self.gap_visualisation.redraw:
                    self.gap_visualisation.draw_gaps_tree_nodes(self.root)
                if self.graph_visualisation.redraw:
                    self.graph_visualisation.draw()

    def _handle_critical_event(self, data):
        """
        Handle critical event
        """
        event = CriticalEventEnum(data.event_type)

        if event == CriticalEventEnum.APPEAR:
            self._handle_appear(data.angle_new_1)
        
        if event == CriticalEventEnum.DISAPPEAR:
            self._handle_disappear(data.angle_old_1)

        if event == CriticalEventEnum.SPLIT:
            self._handle_split(data.angle_old_1, data.angle_new_1, data.angle_new_2)

        if event == CriticalEventEnum.MERGE:
            self._handle_merge(data.angle_new_1, data.angle_old_1, data.angle_old_2)
        self.gap_visualisation.redraw = True
      
    def _handle_move_event(self,data):
        """
        Handle the move event. Update the position of the gap.

        data (GapMove): angle where the gap appeared before and the angle where the gap is now
        """
        angle_old = data.angle_old
        angle_new = data.angle_new

        if self.root[angle_old] == None:
            direction = 0
            if angle_old - angle_new < 0:
                # search negative direction
                direction = -1
            else:
                # search positive direction
                direction = +1

            for i in range(0,5):
                if self.root[(angle_old + (direction * i)) % len(self.root)] != None:
                    angle_old = (angle_old + (direction * i)) % len(self.root)
                    break
        
        if self.root[angle_old] != None:
            self.root[angle_new] = self.root[angle_old]
            self.root[angle_old] = None

        self.gap_visualisation.redraw = True

    def _handle_collection(self,data):
        """
        Handles update on topic collection_critical_and_moved

        data (CollectionCriticalAndMoved):
        """
        self.lock.acquire()
        for event in data.events.events:
            self._handle_critical_event(event)

        for move in data.gap_moves.gap_moves:
            self._handle_move_event(move)

        if len(data.events.events) > 0:
            self._publish_gap_tree()
        
        self.lock.release()

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

        Parameters:
        angle (int): Angle at which the gap disappeared
        """
        if self.root[angle] != None:
            if len(self.root[angle].children) > 0:
                self._recursive_remove_childs_from_graph(self.root[angle])

            # update graph
            self.graph_visualisation.remove_node(self.root[angle])
            self.graph_visualisation.redraw = True

            self.root[angle] = None

    def _recursive_remove_childs_from_graph(self, node):
        """
        Check the node for childs and remove them from the graph

        Parameters:
        node (TreeNode): Node that needs to be removed
        """
        if len(node.children) != 0:
            return
        
        # remove all connected nodes
        for child in nodes.children:
            self._recursive_remove_childs(child)
            self.graph_visualisation.remove_node(child)

    def _handle_split(self, angle_old, angle_new_1, angle_new_2):
        """
        Gap at angle_splits into angle_new_1 and angle_new_2

        Parameters:
        angle_old (int): angle where the spliting disconitnuity is
        angle_new_1 (int): angle of appeared discontinuity from split
        angle_new_2 (int): angle of appeared discontinuity from split
        """        
        angle = None

        # order angles asc
        start = angle_new_1
        end = angle_new_2
        if angle_new_2 < start:
            start = angle_new_2
            end = angle_new_1

        # the node representing the splitted gap has childs if the appeard gaps are kown
        if self.root[angle_old] != None:
            if len(self.root[angle_old].children) == 2:
                self._gap_split_existing(angle_old, angle_new_1, angle_new_2)
            else:
                self._gap_split_new(angle_old, angle_new_1, angle_new_2)
            
    def _gap_split_existing(self, angle_old, angle_new_1, angle_new_2):
        """
        From node at angle_old get the childs and place them at angle_new_1 and angle_new_1.

        Parameters:
        angle_old (int): Angle of the gap that splitted
        angle_new_1 (int): First angle where the first child needs to be placed in the tree
        angle_new_2 (int): Second angle where the second child needs to be placed in the tree
        """
        # get the splitting node
        node_splitting = self.root[angle_old]
        
        # remove node from tree 
        self.root[angle_old] = None

        # remove edge bewteen root and the splitting node
        self.graph_visualisation.remove_edge_between_root_and_node(node_splitting)
        self.graph_visualisation.remove_node(node_splitting)

        # updated the tree by adding the nodes resulted from split
        if angle_new_2 < angle_new_1:
            self.root[angle_new_2] = node_splitting.children[0]
            self.root[angle_new_1] = node_splitting.children[1]
        else:
            self.root[angle_new_1] = node_splitting.children[0]
            self.root[angle_new_2] = node_splitting.children[1]

        # connect the nodes to root
        self.graph_visualisation.add_edge_between_root_and_node(self.root[angle_new_1])
        self.graph_visualisation.add_edge_between_root_and_node(self.root[angle_new_2])

        self.graph_visualisation.redraw = True
    
    def _gap_split_new(self, angle_old, angle_new_1, angle_new_2):
        """
        Create new nodes for angle_new_1 and angle_new_2.

        Parameters:
        angle_old (int): Angle of the gap that splitted
        angle_new_1 (int): First angle for new node
        angle_new_2 (int): Second angle for new node
        """

        # remove node, which represents the splitted gap, from the graph
        self.graph_visualisation.remove_node(self.root[angle_old])
        self.root[angle_old] = None

        # create new nodes and add to the root
        node1 = TreeNode()
        self.root[angle_new_1] = node1

        node2 = TreeNode()
        self.root[angle_new_2] = node2

        # update graph
        self.graph_visualisation.add_node_to_root(self.root[angle_new_1])
        self.graph_visualisation.add_node_to_root(self.root[angle_new_2])
        self.graph_visualisation.redraw = True

    def _handle_merge(self, angle_new, angle_old_1, angle_old_2):
        """
        Gap at angle_old_1 and angle_old_2 merge into angle_new_1.

        Paramters:
        angle_new (int): angle of distcontinuity resulting from merge
        angle_old_1 (int): angle of first discontinuity that merged
        angle_old_2 (int): angle of second discontinuity that merged
        """
        # get the merging nodes
        node_1 = self.root[angle_old_1]
        node_2 = self.root[angle_old_2]
        
        # merge nodes
        node = TreeNode()
        if angle_old_2 < angle_old_1:
            node.children.append(node_2)
            node.children.append(node_1)
        else:
            node.children.append(node_1)
            node.children.append(node_2)
        
        # delete old from depth jumps
        self.graph_visualisation.remove_edge_between_root_and_node(node_1)
        self.root[angle_old_1] = None
        self.graph_visualisation.remove_edge_between_root_and_node(node_2)
        self.root[angle_old_2] = None
        # add the merged node
        self.root[angle_new] = node

        # update graph
        self.graph_visualisation.add_node_to_root(node)
        self.graph_visualisation.add_edge_between_nodes(node, node_1)
        self.graph_visualisation.add_edge_between_nodes(node, node_2)
        self.graph_visualisation.redraw = True

    def _publish_gap_tree(self):
        """
        Publish the gaps visibile from the root on the topic gap_tree.
        """
        ros_msg_nodes = self._convert_root_to_ros_msg()

        self.pub_tree.publish(ros_msg_nodes)

    def _convert_root_to_ros_msg(self):
        """
        Converts the local tree to a ROS message
        """
        # Create the gap tree msg and convert the first layer of root to ros msgs
        ros_msg_nodes = GapTreeNodes()
        
        #root node
        ros_msg_node = GapTreeNode()
        ros_msg_node.id = 0

        ros_msg_nodes.tree_nodes.append(ros_msg_node)

        for node in self.root:
            if node != None:
                ros_msg_node.children_ids.append(node.id)
                ros_msg_nodes.tree_nodes = ros_msg_nodes.tree_nodes + self._convert_node_to_ros_msg(node)
        
        return ros_msg_nodes

    def _convert_node_to_ros_msg(self, node):
        """
        Convertes the given tree node to a ros message tree node
        """
        ros_msg_node = GapTreeNode()
        ros_msg_node.id = node.id

        ros_msg_nodes = []
        ros_msg_nodes.append(ros_msg_node)

        for child in node.children:
            ros_msg_node.children_ids.append(child.id)
            ros_msg_nodes = ros_msg_nodes + self._convert_node_to_ros_msg(child)

        return ros_msg_nodes
        

if __name__ == "__main__":
    try:
        gnt = GapNavigationTree()
        gnt.run()
    except rospy.ROSInterruptException as ex:
        print(str(ex))