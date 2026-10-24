"""Observed Nav2 log events; descriptions, not an inferred failure cause."""
import re,json

def diagnose(run):
 text=re.sub(r'\x1b\[[0-9;]*m','',(run/'launch.log').read_text(errors='replace'))
 lines=text.splitlines();begin=next((i for i,l in enumerate(lines) if 'Begin navigating from current location' in l),len(lines));lines=lines[begin:]
 out={'scope':'Log from accepted NavigateToPose start through process cleanup; raw log retained; event counts do not prove root cause',
      'lattice_search_failed_attempt_count':sum('Lattice attempt' in l and 'found=0' in l for l in lines),
      'body_heading_grid_route_count':sum('Using continuous-footprint body-heading grid route' in l for l in lines),
      'planner_action_aborting_handle_count':sum('planner_server' in l and '[ActionServer] Aborting handle' in l for l in lines),
      'follow_path_client_cancel_count':sum('[follow_path]' in l and 'Client requested to cancel' in l for l in lines),
      'controller_received_path_count':sum('Received a goal, begin computing control effort' in l for l in lines),
      'control_deadline_warning_count':sum('Control loop missed its desired rate' in l for l in lines),
      'stale_tf_message_drop_count':sum('Message Filter dropping message' in l and 'earlier than all the data' in l for l in lines)}
 (run/'navigation_log_diagnostics.json').write_text(json.dumps(out,indent=2));return out
