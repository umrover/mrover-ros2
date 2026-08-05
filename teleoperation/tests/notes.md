Copied over Kevin's tests from skj/test-suite here because merging was beyond my paygrade. 

## Interesting things
- In `test_create_preserves_fields`, 'coverage_radius' doesn't exist in waypoints
    - Also in `test_course_preserves_coverage_radius`
    - 1 or more other functions in test_waypoints_auton_course.py
- `test_clear_all_removes_everything` seems to call an api utility that doesn't exist. Do we need to be able to clear *everything*?
- In ```test_arm_ros_integration.py```, all websocket connection attempts result in a 403 error

### Playwright
- Does an element with id 'pw-controller-status-toggle' exist in DMView?
- Funnel controls are hard to test
- I think many element ids that are being tested for don't exist/are outdated
- Now it seems to crash around test 17 on my machine. Could it be storage space?
- I'm gonna make a seperate md file for these tests

## Tests that are fine but, fail because code doesn't work:
- To my understanding, arm IK is currently WIP
- `test_default_tag_ids_are_0_through_7`
    - `tag_id` is NoneType
    - Are all autonomy waypoints supposed to have tag ids, or did this change? `test_default_tag_ids_are_0_through_7` assumes this.
- `test_get_recording_waypoints`
    -`get_recording_waypoints` in `recordings.py` tries to select 'altitude' (it doesn't exist) 
