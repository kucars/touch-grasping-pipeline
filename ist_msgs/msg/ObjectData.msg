# Handle object type (continuous and discrete)
ist_msgs/ObjectType type

# object categories possibilities
ist_msgs/ObjectCategory[] category_hypotheses

# handle id of closest object
int32 closest_object_id

# optional information about actionable parts on the object
ist_msgs/ActionablePartData[] actionable_parts_data
