import rospy

from critical_event_enum import CriticalEventEnum
from tree_node import TreeNode
from gap_sensor.msg import MovedGaps, GapMove

class GapNavigationTree:
    
    def __init__(self):
        rospy.init_node('gap_navigation_tree')

        self.root = [None] * 360

        self._init_publishers()
        self._init_subscribers()

    def _init_subscribers(self):
        """
        Initialise subscribers
        """
        self.sub_critical_event = rospy.Publisher("critical_event", CriticalEvent, self._receive_critical_event queue_size=5)
        self.sub_gap_move = rospy.Publisher("gap_move", GapMove, self._receive_move_event, queue_size=5)

    def _init_publishers(self):
        """
        Initialise publishersubscribers
        """
        pass

    def run(self):
        """
        """
        pass

    def _receive_critical_event(self, data):
        """
        Receive critical event
        """
        event = CriticalEventEnum(data.event_type)
        #data.angle_old
        #data.angle_new

        if event == CriticalEventEnum.APPEAR:
            self._handle_appear(data.angle_new)
        
        if event == CriticalEventEnum.DISAPPEAR:
            self._handle_disappear(data.angle_new)

        if event == CriticalEventEnum.SPLIT:
            self._handle_split(data.angle_old, data.angle_new)

        if event == CriticalEventEnum.MERGE:
            self._handle_merge(data.angle_old, data.angle_new)

        
    def _receive_move_event(self,data):
        """
        Receive move event
        """
        self.root[data.angle_new] = self.root[data.angle_old]
        self.root[data.angle_old] = None

    def _handle_appear(self, angle):
        """
        Gap appear at angle.
        """
        node = TreeNode()
        #node.appear = True
        #node.appearents_count = 0
        self.root[angle] = node

    def _handle_disappear(self, angle):
        """
        Gap at angle disappear.
        """
        self.root[angle] = None

    def _handle_split(self, angle_old, angle_new):
        """
        Gap at angle_ splits into angle_old and angle_new
        """        
        angle = None

        # check order angles asc
        start = angle_old
        end = angle_new
        if angle_new < start:
            start = angle_new
            end = angle_old

        # find the gap that was splitted
        for i in range(start, (end + 1) % len(self.root)):
            if self.root[i] != None:
                angle = i

        if len(self.gnt_root.depth_jumps[index].nodes) == 2:
            # get the splitting node
            node_splitting = self.root[index]

            # delete 
            self.root[index] = None

            if index_new < index_old:
                self.root[index_new] = node_splitting.nodes[0]
                self.root[index_old] = node_splitting.nodes[1]
            else:
                self.root[index_old] = node_splitting.nodes[0]
                self.root[index_new] = node_splitting.nodes[1]

            #self.root[index_new].critical_event = CriticalEvent.SPLIT
            #self.root[index_new].critical_event_count = 10
            #self.root[index_old].critical_event = CriticalEvent.SPLIT
            #self.root[index_old].critical_event_count = 10
            
        else:
            # create new nodes
            self.gnt_root.depth_jumps[angle] = None

            node1 = TreeNode()
            #node1.critical_event = CriticalEvent.SPLIT
            #node1.critical_event_count = 6
            self.root[angle_old] = node1
            
            node2 = TreeNode()
            #node2.critical_event = CriticalEvent.SPLIT
            #node2.critical_event_count = 6
            self.root[angle_new] = node2

    
    def _handle_merge(self, angle_old, angle_new):
        """
        Gap at angle_old and angle_new merge into angle_new.
        """
        node_1 = self.root[angle_old]
        node_2 = self.root[angle_new]

        node = TreeNode()
        if angle_new < angle_old:
            node.nodes.append(node_2)
            node.nodes.append(node_1)
        else:
            node.nodes.append(node_1)
            node.nodes.append(node_2)
        
        # delete old from depth jumps
        self.root[angle_old] = None
        # add the merged node
        self.root[angle_new] = node


if __name__ == "__main__":
    try:
        gnt = GapNavigationTree()
        gnt.run()
    except rospy.ROSInterruptException as ex:
        print(str(ex))