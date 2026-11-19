# 영속 기록된 최신 진행 상태입니다. Result가 없으면 status/error 필드는 비어 있습니다.
# record_state 값: ACTIVE, FINISHED, INTERRUPTED, RECORDING_FAILED.
# RECORDING_FAILED는 메모리 내 진단이며 영속 저장을 보장하지 않습니다.
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
string cancel_mode
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
