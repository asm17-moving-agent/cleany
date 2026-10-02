# Latest durable progress; status/error fields are empty without a Result.
# record_state: ACTIVE, FINISHED, INTERRUPTED, RECORDING_FAILED.
# RECORDING_FAILED is an in-memory diagnostic; durability is not claimed.
string mission_id
string task_id
string execution_id
string skill_name
string snapshot_id
uint32 object_id
string destination_id
string execution_profile
string record_state
bool has_result
bool human_confirmation_required
uint64 revision
int64 accepted_at_ns
int64 updated_at_ns
int64 evidence_at_ns
string stage
string substage
string[] completed_substages
string failed_substage
string status
string error_code
string failed_stage
string last_completed_stage
string object_state
string placement_state
string selected_arm
bool stop_confirmed
bool arm_recovered
bool retryable
string message
