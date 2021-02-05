### 6DOF POSE ESTIMATION SURGICAL TOOL

This is c++ implementation of **Real-time 6dof pose estimation of endoscopic instruments using printable markers** 
Currenty this implementation capture video stream form camera. This is can experimentation and hence not written in optimised way. 

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

**Note**

`PLease cite this 'Real-time 6dof pose estimation of endoscopic instruments using printable markers' article if you use this code.`
