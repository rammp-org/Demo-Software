import py_trees
from your_bt_file import create_tree  # import your tree


def test_tree(user_input_value):
    tree = create_tree()

    # Manually write to the blackboard instead of using ROS2 subscribers
    client = py_trees.blackboard.Client(name="test")
    client.register_key("user_input", access=py_trees.common.Access.WRITE)
    client.user_input = user_input_value

    # Tick once and check result
    tree.tick()
    print(py_trees.display.unicode_tree(tree.root, show_status=True))
    return tree.root.status


# Test different scenarios
print("--- Test: self level on ---")
result = test_tree("self level on")
assert result == py_trees.common.Status.SUCCESS, "Expected SUCCESS"

print("--- Test: self level off ---")
result = test_tree("self level off")
assert result == py_trees.common.Status.SUCCESS, "Expected SUCCESS"

print("--- Test: invalid input ---")
result = test_tree("invalid command")
assert result == py_trees.common.Status.FAILURE, "Expected FAILURE"

print("All tests passed!")
