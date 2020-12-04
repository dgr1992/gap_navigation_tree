class TreeNode:
    node_count = 0

    def __init__(self):
        self.id = TreeNode.node_count
        TreeNode.node_count = TreeNode.node_count + 1

        self.children = []