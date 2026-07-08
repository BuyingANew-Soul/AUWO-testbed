# Current situational context for excavation (DUL Situational Context Model).
# Derived from the Environment Model + live state; today written directly by
# sim_target_bridge (perception stand-in). Read by the Planner and Tasks.
std_msgs/Header header

bool has_dig
geometry_msgs/Point dig_target      # base_link frame
bool dig_reachable                  # cheap band check; Action IK is authoritative

bool has_dump
geometry_msgs/Point dump_target     # base_link frame
bool dump_reachable
