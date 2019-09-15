## Self-comment
## 个人注释版

*修改了一个我认为错误的地方，旋转矩阵作者使用的是
R*R_f
改为了
R_f*R
### 原因
       * 这里使用的是右乘更新　　Ｔ_wc=T_wr*T_rc
       * |R_f t_f|   | R  t|   | R_f*R R_f*t+t_f|
       * |0   1  |*  |0  1|=   |0     1       |
       * 先更新平移再更新旋转
其余详细参见注释

### Origin README



This is an OpenCV 3.0 based implementation of a monocular visual odometry algorithm.

## Algorithm
Uses Nister's Five Point Algorithm for Essential Matrix estimation, and FAST features, with a KLT tracker.
More details are available [here as a report](http://avisingh599.github.io/assets/ugp2-report.pdf), and
[here as a blog post](http://avisingh599.github.io/vision/monocular-vo/). 

Note that this project is not yet capable of doing reliable relative scale estimation, 
so the scale informaion is extracted from the KITTI dataset ground truth files.

## Demo Video

[![Demo video](http://share.gifyoutube.com/Ke1ope.gif)](http://www.youtube.com/watch?v=homos4vd_Zs)


## Requirements
OpenCV 3.0

## How to compile?
Provided with this repo is a CMakeLists.txt file, which you can use to directly compile the code as follows:
```bash
mkdir build
cd build
cmake ..
make
```

## How to run? 
After compilation, in the build directly, type the following:
```bash
./vo
```
## Before you run
In order to run this algorithm, you need to have either your own data, 
or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).
In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.

## Performance
![Results on the KITTI VO Benchmark](http://avisingh599.github.io/images/visodo/2K.png)

## Contact
For any queries, contact: avisingh599@gmail.com

## License
MIT
