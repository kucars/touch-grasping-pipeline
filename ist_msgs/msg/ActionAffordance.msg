# identifies the type of action to be performed
ist_msgs/Action action

# what should be considered to do the action
# in finger action, is the part the finger must do something on
# in hand_action, is the part that should aligned or touch something else
ist_msgs/ObjectPart usage_part

bool reverse

