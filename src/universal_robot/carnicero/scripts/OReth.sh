sudo -E bash -c "source /opt/ros/noetic/setup.bash && \
                 source ~/catkin_ws/devel/setup.bash && \
                 rosrun carnicero ros_ft_publisher_pysoem.py \
                 _interface:=enxa0cec89e23c0 \
                 _data_type:=int16 \
                 _byte_offset:=0 \
                 _num_values:=6 \
                 _scale:=0.001 \
                 _topic_name:=/mesure_ft \
                 _publish_rate:=50.0"
