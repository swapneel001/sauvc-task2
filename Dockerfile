FROM ros:melodic

CMD ["rosrun package1 task_2_node.py","rosrun rosserial_python serial_node.py /dev/ttyACM0"]

