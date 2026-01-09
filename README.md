# Real-time LeRobot Inference to ROS2 Cartesian Control Bridge

> In this project, we create a connector between Hugging Face's LeRobot  and ROS2. As an example, we use two main open-source robotic arms to demonstrate how it works: the HF SO100 robotic arm and the Annin AR4 robotic arm.
>
> The goals:
> - make it able to use the leader arm of HFSO100 as a real-time leader of AR4
> - make the HFSO100 trained models usable on AR4 directly
>
> To do this, we will use relative workspace mapping, cause the AR4 is much bigger than SO100.
