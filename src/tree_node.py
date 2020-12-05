class TreeNode:
    # 0 is reserved for the root node
    node_count = 1

    def __init__(self):
        self.id = TreeNode.node_count
        TreeNode.node_count = TreeNode.node_count + 1

        self.children = []