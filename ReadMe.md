### 6DOF POSE ESTIMATION SURGICAL TOOL

This is c++ implementation of **Real-time 6dof pose estimation of endoscopic instruments using printable markers.** 

**Brief Abstract**

To accurately navigate surgical instruments around in vivo environments in minimally invasive surgeries through flexible endoscopes or rigid scopes, real-time tracking of surgical instruments is required. It is even challenging to track regular-shaped rigid bodies on a less-distorted general-purpose monocular vision, while most of the surgical instruments or devices used are cylindrical and slim imaged with endoscope distortion in nature. Thus, more dedicated and accurate rigid-body tracking approach for cylindrical endoscopic instruments will be more helpful in intra-operative guidance. In this paper, author present an endoscopic instrument tracking approach with printable markers that consists of a green band and a square with four white and one black circle inscribed in it. The proposed approach can estimate the six degrees of freedom pose, i.e., X, Y, Z, Roll, Pitch, and Yaw, of the imaged instrument with respect to the endoscope. 

---

**Marker**

![Marker](./images/marker.png?raw=true "Title")

Above images shows endoscopic view of marker before and after wrapping on the tool.
(a) Planar printable marker view before wrapping on the tool. (b) A front
view of the tool after wrapping with printable marker.

---

**Marker detection algorithm flow diagram is attached below.** 

![Marker detection algorithm flow diagram](./images/markerdetection.png?raw=true "Title")

*Marker detection algorithm*: In preprocessing block
input image is converted into a binary image, which is further passed to feature
detection, where a square module is detected. Finally, in marker identification
block, the marker is detected with proper orientation.

---

**Results**

![Marker detection algorithm flow diagram](./images/lightconditons.png?raw=true "Title")

Above image shows result of detection of marker under varying lighting conditions.

![Marker detection algorithm flow diagram](./images/occlusionconditions.png?raw=true "Title")

Above image shows result of detection of marker under different occlusion conditions.

---

`Note: Currently this code captures video stream form the camera directly and do processing on the captured image. This is an experimentation code and hence not written in optimised way. Please refer to the paper for detailed algorithm and results. Orginal marker image file is stored in the marker folder`

---

Type following command on terminal to generate .o files

```bash
 $ g++ -c *.cpp
```
Type following command on terminal to link .o files

```bash
 $ g++ -o main *.o `pkg-config --libs opencv`
```
Type following command on terminal to run executable 

```bash
 $ ./main
```

---

`Note: PLease cite this 'Real-time 6dof pose estimation of endoscopic instruments using printable markers' article if you use this code/paper. Use following BibTex to cite article.`

```
@ARTICLE{8574022,  author={A. {Gadwe} and H. {Ren}},  journal={IEEE Sensors Journal},
title={Real-Time 6DOF Pose Estimation of Endoscopic Instruments Using Printable Markers}, 
year={2019},  volume={19},  number={6},
pages={2338-2346},  doi={10.1109/JSEN.2018.2886418}}
```
---