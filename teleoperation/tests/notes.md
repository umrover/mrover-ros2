Copied over Kevin's tests from skj/test-suite here because merging was beyond my paygrade. 

## Tests that are fine but fail because code doesn't work:
- `test_default_tag_ids_are_0_through_7`
- `test_get_recording_waypoints`
  -`get_recording_waypoints` in `recordings.py` tries to select 'altitude' (it doesn't exist) 
