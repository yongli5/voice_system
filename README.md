# voice_system

rosrun voice_system tts_voice_node
rosrun voice_system tuling_arv_node
rosrun tts_voice xf_asr_node
rostopic pub -1 /voice/xf_asr_topic std_msgs/Int32 1

g++ xf_asr.cpp linuxrec.cpp speech_recognizer.cpp -I/opt/ros/kinetic/include -I../include /opt/ros/kinetic/lib/libroscpp.so -lboost_signals -lboost_filesystem /opt/ros/kinetic/lib/librosconsole.so /opt/ros/kinetic/lib/librosconsole_log4cxx.so /opt/ros/kinetic/lib/librosconsole_backend_interface.so -llog4cxx -lboost_regex /opt/ros/kinetic/lib/libxmlrpcpp.so /opt/ros/kinetic/lib/libroscpp_serialization.so /opt/ros/kinetic/lib/librostime.so /opt/ros/kinetic/lib/libcpp_common.so -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lpthread -lconsole_bridge -lmsc -lrt -ldl -lpthread -lasound

