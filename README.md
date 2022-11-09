
# Monocular visual Odometry C++

- Build trajectory from a sequence of frames. 
- Based on Mez algorithm (https://github.com/mez/monocular-visual-odometry)
- Developed by Matheus Ribeiro (matheusrbv99@gmail.com). 
- Related paper: Estimação de posição e orientação de câmeras inteligentes (available in PDF in this GIT)

## Building and running
- OpenCV and OpenCV contrib must be installed: https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/
- CMake must be installed. 
- Only required files: main.cpp and CMakeLists.txt (both available in GIT. build folder must be deleted in the first use). 

- In directory /mono_vo:
1. `mkdir build && cd build`
2. `cmake ..`
3. `make`
4. `./mono_vo`

- To see the FAST features visualized
1. `./mono_vo viz`

### Notes
- A sequence of frames is required. I recorded a video from my cellphone. So, I converted it into a sequence of frames using FFMPEG (use 10 FPS. 30 FPS doesn't work properly). The first frame must be labeled "000000.png".

- Movements during trajectory must be as smooth as possible. Sharp moviments may cause the following error:

*Error terminate called after throwing an instance of 'cv::Exception' what():  OpenCV(4.5.5-dev) /home/matheus-ubuntu/opencv_build/opencv/modules/core/src/matrix.cpp:1175: error: (-13:Image step is wrong) The matrix is not continuous, thus its number of rows can not be changed in function 'reshape'*    

- There is no scale, so there's a lot of drift.  

### Sequence of frames tested
-   Available in: https://drive.google.com/drive/folders/1uidBfJyigr88HjftKUeokp6WbgFuATx2?usp=sharing


- Ambiente controlado
1. Trajetoria 1: Circulo
2. Trajetoria 2: Quadrado
3. Trajetória 3: Angulos de 45º
4. Trajetória 4: Semi-círculos

- Ambiente não controlado
1. Trajetória 1: reta
2. Trajetória 2: curvas
3. Trajetória 3: Retorno acentuado
4. Trajetória 4
5. Trajetória 5

### Other codes (directory \other_codes)
1. camera_calibration: used to calibrate camera and and extract camera parameters
2. CvtGrayNSave: convert images to grayscale and convert. (just for fun)
3. demo_dense: test dense optical flow (just for fun)
4. demo_sparse: test sparse optical flow (just for fun)
5. get_execution_time: get OV system execution time
6. undistort_images: undistort frames, but didn't work. Actually, frame got worse
7. tests: codes used in each test from ambiente controlado and ambiente não controlado

### Useful features using FFMPEG In UBUNTU

- Convert video to a image sequence (I used it to convert all videos into a sequence of frames)

`ffmpeg -i PERCURSO_FINAL.MOV -vf fps=10 %06d.png` 

- Convert video to a image sequence starting from 000179

`ffmpeg -i 2.MOV -vf fps=10 -start_number 000179 %06d.png`

- Convert video format

`ffmpeg -i video.MOV -qscale 0 output.mp4`

- change video resolution keeping quality

`ffmpeg -i input.mp4 -vf scale=1280:720 -preset slow -crf 18 output.mp4`

- Crop video (To crop a 80×60 section, starting from position (200, 100))

`ffmpeg -i in.mp4 -filter:v "crop=80:60:200:100" -c:a copy out.mp4`

- Split video into parts (0: video being (sec); 30: amount (sec))

`ffmpeg -i IMG_1325.MOV  -ss 0 -t 30 RETORNO2.MOV`

- Rotate 90º clockwise (2 = 90CounterClockwise)

`ffmpeg -i in.MOV -vf "transpose=1" out.MOV`

- Rename multiple files (a is the old prefix to be replaced, b is the new replacement)

`mmv a\* b\#1`

### How to use git

1. `git status`
2. `git add -A`
3. `git commit -m "write something"`
4. `git push`

- How to create a new repo
1. `cd /directory/you/want`
2. `cd git init`
3. `git commit -m "write something"`
4. `git remote add origin https... `
5. `git push -u origin master`
