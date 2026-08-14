# HH_260810 - Define stable source-mode values without using free-form strings in downstream safety logic.
uint8 REAL_ONLY=0
uint8 SHADOW=1
uint8 HYBRID_OPTIONAL=2
uint8 HYBRID_REQUIRED=3
uint8 VIRTUAL_ONLY_REQUIRED=4

# HH_260810 - Bind each acceptance decision to its exact source, session, map, transform, and snapshot identity.
std_msgs/Header header
uint8 mode
bool armed
bool accepted
bool available
string reason
string source_node_fqn
string source_writer_gid
string diagnostic_writer_gid
string session_id
uint64 sequence
builtin_interfaces/Time source_stamp
float64 receive_age_sec
builtin_interfaces/Time ttl_deadline
string map_digest
string transform_digest
string serialization_contract
uint32 object_count
string snapshot_digest
builtin_interfaces/Time last_transition_time
uint64 accepted_count
uint64 rejected_count
