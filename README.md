# voice_system

rosrun voice_system tts_voice_node
rosrun voice_system tuling_arv_node
rosrun tts_voice xf_asr_node
rostopic pub -1 /voice/xf_asr_topic std_msgs/Int32 1

