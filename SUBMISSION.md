# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: 
1. ```source /opt/ros/foxy/setup.bash```: This command is used to set up the ROS 2 environment which includes environment variables, paths, configurations required for Ros commands and tools.
2. ```source install/local_setup.bash``` : This command is used in specific after building the workspace. It's script contains spcifically information related to the packages in the workspace

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer:
The ```queue_size``` argument is used while creating a subscriber or a publisher to specify the maximum number of messages that can be stored in the message queue. The ```queue_size``` parameter controls how messages are handled in terms of buffering and potential message loss.

1. For publishers, `queue_size` determines the size of the outgoing message queue. If the rate of publishing messages is higher than the rate at which the subscribers are processing them, messages will be buffered in the queue.
   If the queue becomes full, new messages may start to overwrite the oldest messages in the queue. This can lead to message loss if the subscribers are not able to keep up with the incoming message rate.

2. For subscribers, `queue_size` specifies the size of the incoming message queue. The subscriber pulls messages from this queue.
   If the subscriber cannot keep up with the rate at which messages are being published, the queue may become full. In such cases, the subscriber might drop older messages in the queue to make space for new ones.

Choosing an appropriate ```queue_size``` involves finding a balance between preventing message loss and avoiding excessive memory usage, as larger queues consume more memory.
With a small ```queue_size```, the message queue has limited capacity to store incoming messages whereas a larger queue_size allows for more messages to be buffered in the queue.But, large ```queue_size``` is not required in cases where we nee our systems to be very reactive and responsive. Hence, based on the application, the ```queue_size``` can be set accordingly.

### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer:
1. Calling ```ros2 launch``` in the directory where the launch file:
   In this case, if the were no changes made to other files in the package, we don't need to run ```colcon build``` just for changes made in the launch file.
   Running just ```ros2 launch``` in the directory where the launch file is located will reflect the changes without rebuilding the entire package.

2. Calling ```ros2 launch``` when the launch file is installed with the package:
   If the package has been built and installed using ```colcon build``` and you make the changes to the launch file, we would have to rebuild the entire package again using ```colcon build``` for the changes to be reflected.
   
