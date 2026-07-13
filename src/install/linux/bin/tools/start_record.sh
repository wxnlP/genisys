data='{
    "action_name": "record_joint_state",
    "preparation_duration_s": 0,
    "record_duration_s": 0
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StartRecord' \
    -d "$data"